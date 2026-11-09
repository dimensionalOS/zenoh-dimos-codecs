//! The passthrough video encoder: it forwards an access unit the publisher already encoded instead of encoding one.
//!
//! The gateway's other video encoders take a picture and produce a bitstream. This one takes the bitstream, because for
//! an H.265 camera the decode and re-encode in between is the whole cost: the gateway's H.265 decode is per-sample (see
//! [`compressed_video::to_video`](super::compressed_video::to_video)), so only the keyframes play — about one a second.
//! Handed straight to the browser on a `video-h265` track, every frame plays, at the source's own rate and latency,
//! and the browser's hardware decoder does the work.
//!
//! What this has to supply that the publisher's bitstream does not:
//! - **the picture size**, read off the sequence parameter set ([`h265_sps`](super::h265_sps)) and remembered, because
//!   only keyframes carry one and the gateway wants a size with every frame;
//! - **the parameter sets**, remembered and put back in front of a keyframe that arrives without them, so a viewer that
//!   joins mid-stream can start at the next keyframe rather than waiting for one that happens to carry them.
//!
//! What it deliberately does not do: honour [`VideoTarget`]'s size, bitrate or keyframe request. It did not encode these
//! frames and cannot re-encode them, so the stream costs what the publisher's encoder chose. The gateway knows this
//! ([`VideoFormat::passthrough_only`]) and neither thins nor rescales such a stream; a viewer that needs a keyframe
//! waits for the publisher's next one.

use super::bitstream::Unit;
use anyhow::{Result, anyhow, ensure};
use zenoh_gateway::{DecodedFrame, EncodedVideo, H264Encoder, VideoEncoder, VideoFormat, VideoTarget};

/// Forwards already-encoded H.264/H.265 access units ([`AccessUnit`], from a compressed-video encoding's
/// [`decode`](zenoh_gateway::MessageEncoding::decode) on a video track whose format matches the source codec) to the
/// gateway's track, instead of decoding and re-encoding them. For an already-encoded camera stream that is the whole
/// cost, and passing the bytes through keeps every frame playable (the browser's decoder holds the reference chain)
/// rather than one keyframe a second.
///
/// One per encode session, so its remembered size and parameter sets follow one stream.
#[derive(Debug)]
pub struct Passthrough {
    format: VideoFormat,
    /// the newest size any sequence parameter set gave: frames between keyframes carry none
    size: Option<(u32, u32)>,
    /// the newest headers seen (H.26x parameter sets, AV1's sequence header), to put back in front of a keyframe without them
    headers: Option<Vec<u8>>,
}

/// The H.265 passthrough (the only format that is *always* passed through — there is no H.265 encoder here).
pub type H265Passthrough = Passthrough;

impl Default for Passthrough {
    fn default() -> Self {
        Passthrough::new(VideoFormat::H265)
    }
}

impl Passthrough {
    pub fn new(format: VideoFormat) -> Self {
        Passthrough { format, size: None, headers: None }
    }
}

impl VideoEncoder for Passthrough {
    fn format(&self) -> VideoFormat {
        self.format
    }

    fn encode(&mut self, frame: &DecodedFrame, _target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        let unit: &Unit = frame.downcast().map_err(|error| anyhow!("the {:?} passthrough encoder takes access units: {error:#}", self.format))?;
        ensure!(unit.format == self.format, "the {:?} passthrough encoder was handed {:?} data", self.format, unit.format);
        if let Some(sets) = &unit.headers {
            self.headers = Some(sets.clone());
        }
        if let Some(size) = unit.size {
            self.size = Some(size);
        }
        // a frame before any parameter set says how big the picture is: the gateway would take 0x0 for the stream's
        // source size, so say so instead and let the next keyframe start the stream
        let Some((width, height)) = self.size else {
            return Ok(None);
        };
        // a keyframe is only a keyframe to a browser that gets the parameter sets with it
        let data = match (unit.keyframe, unit.headers.is_none(), &self.headers) {
            (true, true, Some(sets)) => [sets.as_slice(), unit.data.as_slice()].concat(),
            _ => unit.data.clone(),
        };
        let keyframe = unit.keyframe && (unit.headers.is_some() || self.headers.is_some());
        Ok(Some(EncodedVideo { data, width, height, keyframe }))
    }
}

/// The H.264 output encoder: forwards an already-H.264 source access unit unchanged ([`Passthrough`]), and otherwise —
/// an H.265 (or raw) source decoded to a picture — encodes it with the built-in software H.264 encoder
/// ([`H264Encoder`]). So an H.264 camera costs the gateway nothing (zero transcode), while an H.265 camera still
/// transcodes to H.264 for browsers with no HEVC. Which path a frame takes is decided upstream by the stream decoder:
/// an [`AccessUnit`] (H.264 source) is forwarded, a `DecodedFrame::Video` (decoded picture) is encoded.
#[derive(Default)]
pub struct H264Relay {
    passthrough: Passthrough,
    encoder: H264Encoder,
    /// whether the frame `encode` last handled was forwarded (so `last_qp`/`last_frame_id`/`next_output` only follow the
    /// software encoder when it was the one that ran)
    forwarded_last: bool,
}

impl H264Relay {
    pub fn new() -> Self {
        H264Relay { passthrough: Passthrough::new(VideoFormat::H264), encoder: H264Encoder::default(), forwarded_last: false }
    }
}

impl VideoEncoder for H264Relay {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        if matches!(frame, DecodedFrame::Video(_)) {
            self.forwarded_last = false;
            self.encoder.encode(frame, target)
        } else {
            self.forwarded_last = true;
            self.passthrough.encode(frame, target)
        }
    }

    fn last_qp(&self) -> Option<f64> {
        (!self.forwarded_last).then(|| self.encoder.last_qp()).flatten()
    }

    fn last_frame_id(&self) -> Option<u64> {
        (!self.forwarded_last).then(|| self.encoder.last_frame_id()).flatten()
    }

    fn next_output(&mut self, wait: std::time::Duration) -> Result<Option<EncodedVideo>> {
        // a forwarded access unit is never pipelined; the software encoder may trail its input
        if self.forwarded_last {
            Ok(None)
        } else {
            self.encoder.next_output(wait)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::bitstream::unit_of;
    use crate::utils::tests::fixture;

    /// A `DecodedFrame` holding `unit`, as a compressed-video encoding's `decode` returns on a `video-h265` channel.
    fn frame(unit: Unit) -> DecodedFrame {
        DecodedFrame::data(unit)
    }

    #[test]
    fn forwards_a_keyframe_with_its_size_and_bytes_unchanged() {
        let data = fixture("compressed/pattern.h265");
        let unit = unit_of("h265", &data).unwrap();
        let mut encoder = H265Passthrough::default();
        let encoded = encoder.encode(&frame(unit), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        assert_eq!((encoded.width, encoded.height), (320, 240), "the size comes from the SPS, not the target");
        assert!(encoded.keyframe);
        assert_eq!(encoded.data, unit_of("h265", &data).unwrap().data, "the access unit goes out as it came in, start codes normalised");
    }


    #[test]
    fn a_predicted_frame_keeps_the_size_the_keyframe_gave() {
        let mut encoder = H265Passthrough::default();
        encoder.encode(&frame(unit_of("h265", &fixture("compressed/pattern.h265")).unwrap()), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap();
        // a bare TRAIL_R slice: no keyframe, no parameter sets, no size of its own
        let predicted = unit_of("h265", &[0, 0, 0, 1, 0x02, 0x01, 0xaa]).unwrap();
        assert_eq!(predicted.size, None);
        let encoded = encoder.encode(&frame(predicted), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        assert_eq!((encoded.width, encoded.height), (320, 240));
        assert!(!encoded.keyframe, "a predicted frame is not a keyframe");
    }

    #[test]
    fn nothing_goes_out_before_a_parameter_set_says_how_big_the_picture_is() {
        let mut encoder = H265Passthrough::default();
        let predicted = unit_of("h265", &[0, 0, 0, 1, 0x02, 0x01, 0xaa]).unwrap();
        assert!(encoder.encode(&frame(predicted), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().is_none());
    }

    #[test]
    fn puts_remembered_headers_back_in_front_of_a_bare_keyframe() {
        let with_sets = fixture("compressed/pattern.h265");
        let bare = fixture("compressed/pattern_bare.h265");
        let mut encoder = H265Passthrough::default();
        encoder.encode(&frame(unit_of("h265", &with_sets).unwrap()), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap();
        let unit = unit_of("h265", &bare).unwrap();
        assert_eq!(unit.headers, None, "the bare fixture carries none");
        let encoded = encoder.encode(&frame(unit), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        let normalized = unit_of("h265", &bare).unwrap().data;
        assert!(encoded.keyframe, "it is a keyframe once the parameter sets are back");
        assert!(encoded.data.len() > normalized.len(), "the parameter sets were put in front");
        assert!(encoded.data.ends_with(&normalized), "and the keyframe itself follows them");
    }

    #[test]
    fn says_why_it_cannot_take_a_picture_or_another_codec() {
        let mut encoder = H265Passthrough::default();
        let target = VideoTarget::new(64, 64, 100_000, 30.0);
        let picture = DecodedFrame::Video(zenoh_gateway::VideoImage::rgb8(2, 2, vec![0; 12]).unwrap());
        let error = encoder.encode(&picture, &target).unwrap_err().to_string();
        assert!(error.contains("takes access units"), "{error}");
        let h264 = unit_of("h264", &fixture("compressed/pattern.h264")).unwrap();
        let error = encoder.encode(&frame(h264), &target).unwrap_err().to_string();
        assert!(error.contains("was handed H264 data"), "{error}");
    }

    #[test]
    fn h264_relay_forwards_an_h264_source_unchanged() {
        let data = fixture("compressed/pattern.h264");
        let unit = unit_of("h264", &data).unwrap();
        let mut relay = H264Relay::new();
        let encoded = relay.encode(&frame(unit.clone()), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        assert_eq!((encoded.width, encoded.height), (320, 240), "the size comes from the H.264 SPS, not the target");
        assert!(encoded.keyframe);
        assert_eq!(encoded.data, unit.data, "the H.264 access unit goes out as it came in");
        assert_eq!(relay.last_qp(), None, "a forwarded frame has no encoder quantizer");
    }
}
