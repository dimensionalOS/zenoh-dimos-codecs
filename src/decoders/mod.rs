//! Choosing a decoder for a source bitstream: the fastest one **this process** can actually reach.
//!
//! The mirror of [`encoders`](crate::encoders). A stream that is not being forwarded has to be decoded, and which
//! decoder is available depends on the machine, the build and — this is the part that is easy to get wrong — the
//! *process*: on the M20 the Rockchip VPU's devices (`/dev/mpp_service`, `/dev/rga`) are `crw------- root root`, and a
//! GStreamer element that cannot open its device is reported by `gst-inspect` as **absent**, not as present and
//! unusable. The camera stack there runs as root and the hardware works; a probe run as an ordinary user sees nothing.
//!
//! Two rules follow, and both are the point of this module:
//! 1. **Probe where you decode.** The decoder is built in the process that will feed it, never ahead of time in a
//!    helper, because the answer is not a property of the machine.
//! 2. **Say "not available to me", never "not on this machine"**, and name what was looked for. A hardware tier that
//!    silently falls through to software looks exactly like a performance regression, which is the most expensive kind
//!    of wrong log line: it sends someone to profile the encoder when the answer was a device permission.

use crate::utils::bitstream::Bitstream;
use crate::utils::{av1, h264, h265};
use anyhow::{Result, bail};
use zenoh_gateway::{VideoFormat, VideoImage};

/// A decoder that keeps its state across the units of one stream, so predicted frames decode.
///
/// One per stream. The three software decoders behind it ([`h264::Stream`], [`h265::Stream`], [`av1::Stream`]) share a
/// shape deliberately, so this trait is a thin wrapper over each rather than an adapter with logic in it.
pub trait StreamingDecoder: Send {
    /// One coded unit in; the picture it completed, with the `frame_id` of the unit **that picture** came from — which
    /// is an earlier unit's for a decoder that holds frames, and is what lets the gateway stamp a picture with its own
    /// sample's time rather than the newest one's.
    ///
    /// `keyframe`: the unit is decodable on its own. `size`: the picture size from the stream's headers, for a decoder
    /// that cannot report its own (GStreamer's, where nothing in the pipeline gives it back).
    fn decode(&mut self, data: &[u8], keyframe: bool, size: Option<(u32, u32)>, frame_id: u64) -> Result<Option<(VideoImage, u64)>>;

    /// Samples were lost: show nothing until the next keyframe, rather than a picture built on frames never seen.
    ///
    /// Every implementation keeps its decoder. A keyframe has no references, so it makes the picture whole by itself,
    /// which is the whole of what a new decoder would give — and building one per lost sample cost ~35 ms of stall each
    /// where that was measured, on a decoder that was working.
    fn resync(&mut self);

    /// Which decoder this is, for the one line a stream logs about what it got.
    fn name(&self) -> String;

    /// Decode pipelines this decoder has built, where it builds any (GStreamer). **One** per stream is right; a count
    /// that climbs with [`resync`](Self::resync) is that per-gap stall coming back.
    fn pipelines_made(&self) -> u64 {
        0
    }

    /// How many units may be inside before a [`decode`](Self::decode) waits for its picture.
    ///
    /// `0` waits for every one: one decode and one encode in series per frame, which is what a throughput measurement
    /// wants (every picture comes out of the call that fed it) and what capped a 720p stream at ~22 fps on the robot.
    fn hold(&mut self, _frames: u64) {}
}

/// The fastest decoder this process can reach for `bitstream`'s format, having said in the log which it is and why.
///
/// The log line is not decoration: the difference between the tiers is several times the frame rate, so a stream that
/// quietly got the slow one is a performance bug with no symptom but the frame rate. Every branch below says what it
/// picked, and every branch that *fell through* says what it could not have.
pub fn select(bitstream: &dyn Bitstream) -> Result<Box<dyn StreamingDecoder>> {
    select_with(bitstream, crate::settings::settings().force_libde265)
}

/// [`select`] with the choice passed in, so one process can measure two decoders against each other — the settings are
/// fixed for a process once read, so a test cannot flip them.
pub fn select_with(bitstream: &dyn Bitstream, force_libde265: bool) -> Result<Box<dyn StreamingDecoder>> {
    let format = bitstream.format();
    #[cfg(feature = "gstreamer")]
    let gstreamer_note = match gstreamer_decoder(format, force_libde265) {
        Ok(Some(decoder)) => {
            log::info!("{format:?} stream decoding with {}", decoder.name());
            return Ok(decoder);
        }
        // nothing to say: this build drives no GStreamer decoder for this format, which is not a fallback
        Ok(None) => String::new(),
        Err(error) => format!(" (no GStreamer {format:?} decoder available to this process: {error:#})"),
    };
    #[cfg(not(feature = "gstreamer"))]
    let gstreamer_note = " (this build has no `gstreamer` feature, so no hardware decoder can be reached)".to_owned();
    let _ = force_libde265;

    let software = software_decoder(format)?;
    log::info!("{format:?} stream decoding with {}{gstreamer_note}", software.name());
    Ok(software)
}

/// The GStreamer decoder for `format`, or `None` where this crate drives none for it.
///
/// Built here, in the process that will decode, for the reason in this module's docs.
#[cfg(feature = "gstreamer")]
fn gstreamer_decoder(format: VideoFormat, force_libde265: bool) -> Result<Option<Box<dyn StreamingDecoder>>> {
    match format {
        // asking for libde265 is asking to measure the two against each other, so GStreamer is skipped rather than
        // picked and then overridden
        VideoFormat::H265 if force_libde265 => {
            bail!("force_libde265 asks for libde265, and GStreamer's is faster where it is installed")
        }
        VideoFormat::H265 => Ok(Some(Box::new(Gstreamer(crate::encoders::gstreamer::H265Decoder::new()?)))),
        // H.264 and AV1 have no GStreamer decoder here *yet*: their software decoders need no system library at all
        // (openh264 and rav1d are both built into this crate), so unlike H.265 there is no machine where the
        // difference is "decodes or does not". Adding them is an element name and a caps string.
        _ => Ok(None),
    }
}

/// The software decoder for `format`, which is the one that works with nothing installed.
fn software_decoder(format: VideoFormat) -> Result<Box<dyn StreamingDecoder>> {
    match format {
        VideoFormat::H264 => Ok(Box::new(Openh264(h264::Stream::new()?))),
        // the only one of the three that needs a system library, loaded with dlopen: there is no vendored H.265
        // decoder here, so this is also the one format a static musl build can be left without
        VideoFormat::H265 => Ok(Box::new(Libde265(h265::Stream::new().map_err(|error| {
            anyhow::anyhow!("no H.265 decoder: neither a GStreamer one nor libde265 ({error:#})")
        })?))),
        VideoFormat::Av1 => Ok(Box::new(Rav1d(av1::Stream::new()?))),
        other => bail!("nothing here decodes {other:?}, so a {other:?} source can only be forwarded to a {other:?} track"),
    }
}

/// GStreamer's, by default libav's `avdec_h265`: ~91 fps of 720p on an RK3588 A76 core against libde265's ~50, and the
/// only software H.265 fast enough on that hardware. Set to `mppvideodec` where the VPU is reachable.
#[cfg(feature = "gstreamer")]
struct Gstreamer(crate::encoders::gstreamer::H265Decoder);

/// libde265, loaded with dlopen: the fallback where GStreamer is not installed.
struct Libde265(h265::Stream);

/// openh264, vendored C that this crate already builds for the encoder: available everywhere, musl included.
struct Openh264(h264::Stream);

/// rav1d, the pure-Rust port of dav1d: no system library and no C toolchain, so available everywhere.
struct Rav1d(av1::Stream);

#[cfg(feature = "gstreamer")]
impl StreamingDecoder for Gstreamer {
    fn decode(&mut self, data: &[u8], keyframe: bool, size: Option<(u32, u32)>, frame_id: u64) -> Result<Option<(VideoImage, u64)>> {
        // GStreamer is handed the size because nothing in the pipeline reports it back; before any headers have said
        // what it is there is also no keyframe to start from, so there is nothing to decode yet
        let Some(size) = size else { return Ok(None) };
        // a fresh pipeline cannot use a predicted frame, and pushing one anyway costs a full output timeout waiting for
        // a picture that will never come — seconds per frame, far worse than the second of pictures a resync costs
        if self.0.needs_keyframe() && !keyframe {
            return Ok(None);
        }
        let Some((data, frame_id)) = self.0.decode(data, size, frame_id)? else { return Ok(None) };
        VideoImage::i420(size.0, size.1, data).map(|picture| Some((picture, frame_id)))
    }

    fn resync(&mut self) {
        self.0.resync();
    }

    fn name(&self) -> String {
        format!("gstreamer {}", self.0.element())
    }

    fn pipelines_made(&self) -> u64 {
        self.0.pipelines_made()
    }

    fn hold(&mut self, frames: u64) {
        self.0.hold(frames);
    }
}

/// The three software decoders hold no frames and report their own picture size, so each is the same three lines over a
/// `push(data, keyframe)`: the picture is the unit just fed, and `size` is of no use to them.
macro_rules! software_decoder {
    ($wrapper:ty, $name:literal) => {
        impl StreamingDecoder for $wrapper {
            fn decode(&mut self, data: &[u8], keyframe: bool, _size: Option<(u32, u32)>, frame_id: u64) -> Result<Option<(VideoImage, u64)>> {
                Ok(self.0.push(data, keyframe)?.map(|picture| (picture, frame_id)))
            }

            fn resync(&mut self) {
                self.0.resync();
            }

            fn name(&self) -> String {
                $name.to_owned()
            }
        }
    };
}

software_decoder!(Libde265, "libde265");
software_decoder!(Openh264, "openh264");
software_decoder!(Rav1d, "rav1d");

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::bitstream;
    use crate::utils::tests::fixture;

    /// Every format this crate reads has a decoder, so a source of it can be transcoded and not only forwarded. This is
    /// the claim that makes the transcoding matrix more than one cell wide, and it is one call per codec.
    #[test]
    fn every_bitstream_this_reads_has_a_decoder() {
        for bitstream in bitstream::ALL {
            match select(*bitstream) {
                Ok(decoder) => eprintln!("{:?}: {}", bitstream.format(), decoder.name()),
                // libde265 is loaded with dlopen and is not on every machine; the other two are built in, so only
                // H.265 may legitimately have none here
                Err(error) if bitstream.format() == VideoFormat::H265 => eprintln!("H265: none on this machine ({error:#})"),
                Err(error) => panic!("{:?} has no decoder: {error:#}", bitstream.format()),
            }
        }
    }

    /// A format the gateway can send but this crate cannot decode says so in terms of what it means for the stream —
    /// that such a source can only be forwarded — rather than as a missing-decoder error.
    #[test]
    fn a_format_with_no_decoder_says_what_that_means_for_the_stream() {
        // `map` because a boxed `dyn StreamingDecoder` has no `Debug`, which `unwrap_err` would want of the Ok side
        let error = software_decoder(VideoFormat::Vp8).map(|_| ()).unwrap_err().to_string();
        assert!(error.contains("can only be forwarded"), "{error}");
    }

    /// Each decoder really decodes its own codec's keyframe through the trait, which is what says the wrappers are
    /// wired to the decoder their name claims.
    #[test]
    fn each_decoder_decodes_its_own_keyframe() {
        for (name, file) in [("h264", "compressed/pattern.h264"), ("h265", "compressed/pattern.h265"), ("av1", "compressed/pattern.av1")] {
            let bitstream = bitstream::by_name(name).unwrap();
            let unit = bitstream.unit(&fixture(file)).unwrap();
            let Ok(mut decoder) = select(bitstream) else {
                eprintln!("skipping {name}: no decoder on this machine");
                continue;
            };
            decoder.hold(0);
            let (picture, frame_id) = decoder.decode(&unit.data, unit.keyframe, unit.size, 7).unwrap().expect("the keyframe decodes");
            assert_eq!((picture.width(), picture.height()), (320, 240), "{name} with {}", decoder.name());
            assert_eq!(frame_id, 7, "{name}: holding nothing, the picture is the unit just fed");
        }
    }
}
