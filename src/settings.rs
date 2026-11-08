//! What a deployment chooses about how this crate decodes and encodes.
//!
//! These were environment variables, which made them invisible: a robot behaved differently from the same binary
//! and nothing in its configuration said why. They are arguments now. A program that embeds this crate reads them
//! from wherever it keeps its own configuration and calls [`configure`] once, before any video starts; anything it
//! does not set keeps the default, which is what every machine ran on before these existed.

use std::sync::OnceLock;

/// Which pipelines get a core to themselves.
///
/// **`Decoders` is the shape that was measured to work.** A decode is the expensive stage (12 ms of a 33 ms interval
/// for 720p H.265 in software) and it is the one that needs a core of its own; an encode is a few milliseconds and
/// places itself perfectly well on what is left.
///
/// Pinning encoders too was tried and is worse: a GStreamer encoder has **two** threads of its element's name —
/// `x264enc` keeps the appsrc's streaming task as well as its own, and `threads=1` caps only the latter — so the pin
/// confines both to one core, which took an encode from 7 ms to 24-55 ms and two cameras from 29 fps to 15-18.
/// `All` is kept for a machine where that turns out differently.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum PinPipelines {
    /// the scheduler places everything
    #[default]
    Off,
    /// decoders only
    Decoders,
    /// decoders and encoders both
    All,
}

/// How many frames an encoder may hold before its output is waited for. Lower trades throughput for latency.
const GST_IN_FLIGHT: usize = 3;
/// The JPEG pipeline's own figure, which is higher because its frames are cheap.
const GST_JPEG_IN_FLIGHT: usize = 6;
/// Frames an H.265 decoder keeps inside itself. One lets a decode overlap the next arrival; zero waits for every
/// picture, which costs about a frame interval of latency but holds nothing back.
const H265_FRAMES_INSIDE: usize = 1;
/// Worker threads per libde265 decoder.
const H265_THREADS: usize = 2;
/// libav's HEVC decoder, which is faster than libde265 and present wherever GStreamer's libav plugin is.
pub const DEFAULT_H265_DECODER: &str = "avdec_h265";

/// How this crate should decode and encode. Build with [`Settings::default`] and change what you mean to change.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Settings {
    /// A GStreamer element to encode video with (`x264enc`, `vp8enc`, a hardware encoder), tried before falling
    /// back to this crate's own software H.264. `None` leaves the choice to the usual search.
    pub video_encoder: Option<String>,
    /// Frames a GStreamer encoder may hold before its output is waited for.
    pub gst_in_flight: usize,
    /// The same for the JPEG pipeline.
    pub gst_jpeg_in_flight: usize,
    /// Frames an H.265 decoder keeps inside itself; `0` waits for every picture.
    pub h265_frames_inside: usize,
    /// The GStreamer element an H.265 decoder uses, with properties: `"avdec_h265 max-threads=1"`.
    pub h265_gst_decoder: String,
    /// Which pipelines are pinned to a core of their own.
    pub pin_pipelines: PinPipelines,
    /// Worker threads a libde265 decoder gets, never more than this machine has. Two is the knee: it buys half
    /// again as much for one extra thread, while four would give two cameras eight threads and leave the encoders
    /// nothing.
    pub h265_threads: usize,
    /// Force the libde265 H.265 decoder instead of the fastest this machine has, to compare the two on one machine.
    /// GStreamer's `avdec_h265` is nearly twice as quick where it is installed (~83 fps of 720p against ~50 on an
    /// RK3588 core), so this costs real frames and exists only for measuring.
    pub force_libde265: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            video_encoder: None,
            gst_in_flight: GST_IN_FLIGHT,
            gst_jpeg_in_flight: GST_JPEG_IN_FLIGHT,
            h265_frames_inside: H265_FRAMES_INSIDE,
            h265_gst_decoder: DEFAULT_H265_DECODER.to_owned(),
            pin_pipelines: PinPipelines::Off,
            h265_threads: H265_THREADS,
            force_libde265: false,
        }
    }
}

impl PinPipelines {
    /// Reads what a configuration file says: `""`, `0`, `off`, `no`, `false` are off; `all`, `both`, `encode`,
    /// `encoders` include encoders; anything else means decoders only — so `1`, the obvious thing to write, gets
    /// the shape that was measured to work rather than the one that was not. A typo should not decide this either,
    /// which is why nothing here is an error.
    pub fn parse(value: &str) -> Self {
        match value.trim().to_ascii_lowercase().as_str() {
            "" | "0" | "off" | "no" | "false" => Self::Off,
            "all" | "both" | "encode" | "encoders" => Self::All,
            _ => Self::Decoders,
        }
    }
}

static SETTINGS: OnceLock<Settings> = OnceLock::new();

/// Sets what this crate does, once, before any video starts.
///
/// Returns the settings back as `Err` if they were already set (or already read, which fixes them at the default) —
/// changing them underneath a running encoder would mean one picture encoded two ways.
pub fn configure(settings: Settings) -> Result<(), Settings> {
    SETTINGS.set(settings)
}

/// What was configured, or the defaults. Reading fixes them, so a later [`configure`] is refused rather than
/// silently ignored.
pub(crate) fn settings() -> &'static Settings {
    SETTINGS.get_or_init(Settings::default)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_defaults_are_what_ran_before_any_of_this_was_configurable() {
        let settings = Settings::default();
        assert_eq!(settings.video_encoder, None);
        assert_eq!(settings.gst_in_flight, 3);
        assert_eq!(settings.gst_jpeg_in_flight, 6);
        assert_eq!(settings.h265_frames_inside, 1);
        assert_eq!(settings.h265_gst_decoder, "avdec_h265");
        assert_eq!(settings.pin_pipelines, PinPipelines::Off);
        assert_eq!(settings.h265_threads, 2);
        assert!(
            !settings.force_libde265,
            "the fastest decoder this machine has, unless told otherwise"
        );
    }

    #[test]
    fn what_a_config_file_might_say_about_pinning() {
        // `1` has to mean decoders only: pinning encoders too took an encode from 7 ms to 24-55 ms
        assert_eq!(PinPipelines::parse("1"), PinPipelines::Decoders);
        for off in ["", "0", "off", "no", "false", "OFF", "  off  "] {
            assert_eq!(PinPipelines::parse(off), PinPipelines::Off, "{off:?}");
        }
        for all in ["all", "both", "encode", "encoders", "ALL"] {
            assert_eq!(PinPipelines::parse(all), PinPipelines::All, "{all:?}");
        }
        // a typo lands on the safe shape rather than failing a robot's startup
        assert_eq!(PinPipelines::parse("yes please"), PinPipelines::Decoders);
    }

    #[test]
    fn pinning_is_off_unless_a_deployment_asks() {
        // which core a robot's video runs on is the deployment's decision, not this crate's
        assert_eq!(PinPipelines::default(), PinPipelines::Off);
    }
}
