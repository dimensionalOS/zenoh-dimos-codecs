//! Hardware video encoders for [zenoh-gateway](https://github.com/dimensionalOS/zenoh-gateway), as [`VideoEncoder`](zenoh_gateway::VideoEncoder)s for
//! [`ServerBuilder::video_encoder`](zenoh_gateway::ServerBuilder::video_encoder):
//! - `videotoolbox` (feature): macOS VideoToolbox, the Mac's media engine;
//! - `gstreamer` (feature): a GStreamer hardware encoder found at runtime, `nvv4l2h264enc` (Jetson), `nvh264enc`
//!   (NVENC) or `vah264enc` / `vaapih264enc` (VAAPI).
//!
//! [`select`](crate::encoders::select) probes them (each must encode a test frame) and wraps the one it picks in a [`Fallback`](crate::encoders::Fallback) to software
//! H.264, so a hardware encoder that fails mid-stream hands over to openh264 from a keyframe.
//!
//! ```no_run
//! # async fn run() -> anyhow::Result<()> {
//! let selected = zenoh_dimos_codecs::encoders::select(zenoh_dimos_codecs::encoders::Backend::Auto)?;
//! let mut builder = zenoh_gateway::Server::builder();
//! if let Some(factory) = selected.factory {
//!     builder = builder.video_encoder(factory);
//! }
//! println!("video encoder: {}", selected.name);
//! # Ok(())
//! # }
//! ```

#[cfg(feature = "gstreamer")]
pub mod gstreamer;
// only GStreamer's pipelines have a streaming thread to place
#[cfg(feature = "gstreamer")]
mod pin;
#[cfg(all(feature = "videotoolbox", target_os = "macos"))]
pub mod videotoolbox;
pub mod process;

use anyhow::{Result, bail};
use std::str::FromStr;
use zenoh_gateway::{DecodedFrame, EncodedVideo, H264Encoder, VideoEncoder, VideoFormat, VideoTarget};

/// Keyframe at least this often (seconds), on top of the browser's PLI/FIR: zenoh-gateway's `VideoPolicy::keyframe_interval`
/// default, for its reason (every 3 s, each keyframe took 0.3-0.5 s to cross a 1 Mbit/s link and held the frames behind it).
#[cfg(any(all(feature = "videotoolbox", target_os = "macos"), feature = "gstreamer"))]
const KEYFRAME_SECONDS: f64 = 60.0;

/// The frame and target a hardware encoder must encode to be picked: mid-gray 320x240 at 1 Mbit/s, 30 fps.
#[cfg(any(all(feature = "videotoolbox", target_os = "macos"), feature = "gstreamer"))]
fn probe_frame() -> Result<(DecodedFrame, VideoTarget)> {
    let image = zenoh_gateway::VideoImage::i420(320, 240, vec![128; 320 * 240 * 3 / 2])?;
    Ok((DecodedFrame::Video(image), VideoTarget::new(320, 240, 1_000_000, 30.0)))
}

/// Which encoder to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Backend {
    /// The first hardware one that works (VideoToolbox, then GStreamer), else software.
    Auto,
    /// openh264 (zenoh-gateway's own).
    Software,
    /// macOS VideoToolbox.
    VideoToolbox,
    /// A GStreamer hardware encoder.
    Gstreamer,
}

impl FromStr for Backend {
    type Err = String;

    fn from_str(name: &str) -> Result<Self, String> {
        Ok(match name {
            "auto" => Backend::Auto,
            "software" => Backend::Software,
            "videotoolbox" => Backend::VideoToolbox,
            "gstreamer" => Backend::Gstreamer,
            _ => return Err(format!("unknown video encoder {name:?} (auto, software, videotoolbox, gstreamer)")),
        })
    }
}

/// Makes an encoder per encode session.
pub type Factory = Box<dyn Fn() -> Box<dyn VideoEncoder> + Send + Sync>;

/// What [`select`] picked.
pub struct Selected {
    /// e.g. `"videotoolbox"`, `"gstreamer nvv4l2h264enc"`, `"software"`
    pub name: String,
    /// `None` for software: zenoh-gateway's default.
    pub factory: Option<Factory>,
    /// The hardware encoder alone, without `factory`'s hand-over to software: errors come out instead, for a host that
    /// runs it in a child process and restarts that on an error or a hang (zenoh-gateway's
    /// `ServerBuilder::video_encoder_process`, where software is too slow to be useful, as on a Jetson).
    pub hardware: Option<Factory>,
}

/// The GStreamer element [`Settings::video_encoder`](crate::settings::Settings::video_encoder) names, tried before falling back to software.
///
/// It exists because on some machines there is no hardware encoder and the software default is too slow: an RK3588 has
/// no V4L2 encoder nodes, no VAAPI and no NVENC, so [`select`] reaches zenoh-gateway's openh264 — about 27 fps of 720p
/// on one of its big cores, under the 30 a camera needs — while GStreamer's `x264enc` on the same core does ~84.
///
/// It is not tried unless asked for. `x264enc` is GPL, where openh264 is BSD-licensed with Cisco's patent grant, so
/// which encoder a build uses is a decision for whoever ships it and not one to make by default. Naming the element
/// rather than a backend also makes this the way to pin any encoder, e.g. to compare two on one machine.
/// `" (no hardware one: ...)"` when hardware was looked for and not found, else nothing: a host that asked for
/// software never looked, and saying "no hardware encoder" there would be a puzzle rather than a reason.
fn hardware_note(failures: &[String]) -> String {
    if failures.is_empty() { String::new() } else { format!(" (no hardware one: {})", failures.join("; ")) }
}

fn named_element() -> Option<String> {
    let name = crate::settings::settings().video_encoder.as_deref()?.trim();
    (!name.is_empty()).then(|| name.to_owned())
}

/// Probes `backend` (with [`Backend::Auto`], each hardware one in turn) by encoding a test frame. An explicitly named
/// backend that isn't built in or doesn't work is an error; `Auto` and `Software` fall back to the element
/// the settings name ([`named_element`]), else zenoh-gateway's own.
pub fn select(backend: Backend) -> Result<Selected> {
    let software = || Selected { name: "software".into(), factory: None, hardware: None };
    let candidates: &[Backend] = match backend {
        // `Software` means "no hardware encoder", which is exactly when naming one matters: it is the choice between
        // openh264 and whatever else is installed. Asking for an element and being given openh264 without a word
        // (which is what a host passing `Software` used to get) is the worst of the three outcomes.
        Backend::Software => &[],
        Backend::Auto => &[Backend::VideoToolbox, Backend::Gstreamer],
        Backend::VideoToolbox => &[Backend::VideoToolbox],
        Backend::Gstreamer => &[Backend::Gstreamer],
    };
    let mut failures = Vec::new();
    for candidate in candidates {
        match probe(*candidate) {
            Ok(selected) => return Ok(selected),
            Err(error) => failures.push(format!("{error:#}")),
        }
    }
    if backend == Backend::Auto || backend == Backend::Software {
        // an element asked for by name: on a machine with no hardware encoder this is the only way past openh264
        #[cfg(feature = "gstreamer")]
        if let Some(name) = named_element() {
            match probe_element(&name) {
                Ok(selected) => {
                    log::info!("video encoder: {} as the settings ask{}", selected.name, hardware_note(&failures));
                    return Ok(selected);
                }
                // loudly: someone naming an encoder meant it, and silently encoding with another is how an hour goes
                Err(error) => log::warn!("video_encoder={name} cannot encode ({error:#}): software H.264 instead"),
            }
        }
        // a build without the `gstreamer` feature cannot make the element asked for; saying so beats encoding with
        // another and leaving someone to wonder why the one they named made no difference
        #[cfg(not(feature = "gstreamer"))]
        if let Some(name) = named_element() {
            log::warn!("video_encoder={name} needs this crate's `gstreamer` feature, which this build does not have: software H.264 instead");
        }
        log::info!("video encoder: software H.264{}", hardware_note(&failures));
        return Ok(software());
    }
    bail!("{}", failures.join("; "))
}

/// The GStreamer element called `name`, having encoded a test frame with it. Unlike [`probe`]'s hardware backends this
/// hands over to software on a later failure (`Fallback`) rather than reporting one: a software element that stops
/// working is not a wedged device, and there is nothing better to restart it as.
#[cfg(feature = "gstreamer")]
fn probe_element(name: &str) -> Result<Selected> {
    let (frame, target) = probe_frame()?;
    gstreamer::GstreamerEncoder::with_element(name)?.encode(&frame, &target)?;
    let owned = name.to_owned();
    let factory: Factory = Box::new(move || match gstreamer::GstreamerEncoder::with_element(&owned) {
        Ok(encoder) => Box::new(Fallback::new("gstreamer", encoder)),
        Err(error) => {
            log::warn!("{owned}: {error:#}; software H.264 instead");
            Box::new(H264Encoder::default())
        }
    });
    Ok(Selected { name: format!("gstreamer {name}"), factory: Some(factory), hardware: None })
}

fn probe(backend: Backend) -> Result<Selected> {
    match backend {
        #[cfg(all(feature = "videotoolbox", target_os = "macos"))]
        Backend::VideoToolbox => {
            let (frame, target) = probe_frame()?;
            videotoolbox::VideoToolboxEncoder::default().encode(&frame, &target)?;
            Ok(Selected {
                name: "videotoolbox".into(),
                factory: Some(Box::new(|| Box::new(Fallback::new("videotoolbox", videotoolbox::VideoToolboxEncoder::default())))),
                hardware: Some(Box::new(|| Box::new(videotoolbox::VideoToolboxEncoder::default()))),
            })
        }
        #[cfg(feature = "gstreamer")]
        Backend::Gstreamer => {
            let element = gstreamer::GstreamerEncoder::hardware()?.element();
            let factory: Factory = Box::new(move || match gstreamer::GstreamerEncoder::with_element(element) {
                Ok(encoder) => Box::new(Fallback::new(element, encoder)),
                Err(error) => {
                    log::warn!("{element}: {error:#}; software H.264 instead");
                    Box::new(H264Encoder::default())
                }
            });
            let hardware: Factory = Box::new(move || match gstreamer::GstreamerEncoder::with_element(element) {
                Ok(encoder) => Box::new(encoder),
                Err(error) => Box::new(Failed(format!("{element}: {error:#}"))),
            });
            Ok(Selected { name: format!("gstreamer {element}"), factory: Some(factory), hardware: Some(hardware) })
        }
        other => bail!("{other:?} is not built in (cargo feature {:?})", format!("{other:?}").to_lowercase()),
    }
}

/// A hardware encoder that could not be made: every encode fails with why (a child process running it exits and is
/// started again, which may find the device working).
#[cfg(feature = "gstreamer")]
struct Failed(String);

#[cfg(feature = "gstreamer")]
impl VideoEncoder for Failed {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn encode(&mut self, _: &DecodedFrame, _: &VideoTarget) -> Result<Option<EncodedVideo>> {
        bail!("{}", self.0)
    }
}

/// A hardware encoder that hands over to software H.264 for good after its first error (from a keyframe).
pub struct Fallback<E> {
    name: &'static str,
    hardware: Option<E>,
    software: H264Encoder,
}

impl<E: VideoEncoder> Fallback<E> {
    /// Wraps `hardware` (an H.264 encoder) called `name` in the logs.
    pub fn new(name: &'static str, hardware: E) -> Self {
        Fallback { name, hardware: Some(hardware), software: H264Encoder::default() }
    }
}

impl<E: VideoEncoder> VideoEncoder for Fallback<E> {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        if let Some(hardware) = &mut self.hardware {
            match hardware.encode(frame, target) {
                Ok(encoded) => return Ok(encoded),
                Err(error) => {
                    log::warn!("{}: {error:#}; software H.264 from here on", self.name);
                    self.hardware = None;
                }
            }
        }
        // a new software encoder starts with a keyframe
        self.software.encode(frame, target)
    }

    fn last_frame_id(&self) -> Option<u64> {
        self.hardware.as_ref().and_then(|hardware| hardware.last_frame_id())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zenoh_gateway::VideoImage;

    /// Four solid quadrants: red, green / blue, white.
    fn quadrants(width: u32, height: u32) -> DecodedFrame {
        let pixels = (0..height).flat_map(|y| (0..width).flat_map(move |x| match (y < height / 2, x < width / 2) {
            (true, true) => [255, 0, 0],
            (true, false) => [0, 255, 0],
            (false, true) => [0, 0, 255],
            (false, false) => [255, 255, 255],
        }));
        DecodedFrame::Video(VideoImage::rgb8(width, height, pixels.collect()).unwrap())
    }

    struct Broken;

    impl VideoEncoder for Broken {
        fn format(&self) -> VideoFormat {
            VideoFormat::H264
        }

        fn encode(&mut self, _: &DecodedFrame, _: &VideoTarget) -> Result<Option<EncodedVideo>> {
            bail!("the device went away")
        }
    }

    #[test]
    fn backends_by_name() {
        assert_eq!("auto".parse::<Backend>().unwrap(), Backend::Auto);
        assert_eq!("gstreamer".parse::<Backend>().unwrap(), Backend::Gstreamer);
        assert!("nvenc".parse::<Backend>().is_err());
        let software = select(Backend::Software).unwrap();
        assert!(software.name == "software" && software.factory.is_none());
    }

    #[test]
    fn a_failing_encoder_hands_over_to_software_from_a_keyframe() {
        let mut encoder = Fallback::new("broken", Broken);
        let target = VideoTarget::new(64, 48, 500_000, 30.0);
        let first = encoder.encode(&quadrants(64, 48), &target).unwrap().unwrap();
        assert!(first.keyframe && first.data.starts_with(&[0, 0, 0, 1]));
        assert!(!encoder.encode(&quadrants(64, 48), &target).unwrap().unwrap().keyframe, "software from then on");
    }

    /// Decodes Annex B H.264 and returns the RGB pixel at (x, y) of the last picture.
    #[cfg(all(feature = "videotoolbox", target_os = "macos"))]
    fn decoded_pixel(stream: &[u8], (width, height): (usize, usize), (x, y): (usize, usize)) -> [u8; 3] {
        use openh264::formats::YUVSource;
        let mut decoder = openh264::decoder::Decoder::new().unwrap();
        let mut last = None;
        for packet in openh264::nal_units(stream) {
            if let Ok(Some(picture)) = decoder.decode(packet) {
                assert_eq!(picture.dimensions(), (width, height));
                let mut rgb = vec![0u8; width * height * 3];
                picture.write_rgb8(&mut rgb);
                last = Some(rgb);
            }
        }
        let rgb = last.expect("a decoded picture");
        let at = (y * width + x) * 3;
        [rgb[at], rgb[at + 1], rgb[at + 2]]
    }

    #[cfg(all(feature = "videotoolbox", target_os = "macos"))]
    #[test]
    fn videotoolbox_encodes_decodable_h264() {
        let mut encoder = videotoolbox::VideoToolboxEncoder::default();
        let (frame, mut target) = (quadrants(320, 240), VideoTarget::new(320, 240, 2_000_000, 30.0));
        let mut stream = Vec::new();
        let mut keyframes = Vec::new();
        for index in 0..6 {
            if index == 3 {
                target.bitrate_bps = 4_000_000;
            }
            target.keyframe = index == 5;
            let encoded = encoder.encode(&frame, &target).unwrap().expect("one frame out per frame in");
            assert_eq!((encoded.width, encoded.height), (320, 240));
            assert!(encoded.data.starts_with(&[0, 0, 0, 1]));
            keyframes.push(encoded.keyframe);
            stream.extend(encoded.data);
        }
        assert_eq!(keyframes, [true, false, false, false, false, true], "first, then only when asked (a new bitrate applies in place)");
        let red = decoded_pixel(&stream, (320, 240), (40, 40));
        assert!(red[0] > 200 && red[1] < 60 && red[2] < 60, "red quadrant decodes red: {red:?}");
        let white = decoded_pixel(&stream, (320, 240), (280, 200));
        assert!(white.iter().all(|&channel| channel > 220), "{white:?}");
        let selected = select(Backend::VideoToolbox).unwrap();
        assert_eq!(selected.name, "videotoolbox");
        let encoded = (selected.factory.unwrap())().encode(&frame, &VideoTarget::new(320, 240, 1_000_000, 30.0)).unwrap().unwrap();
        assert!(encoded.keyframe);
    }
}
