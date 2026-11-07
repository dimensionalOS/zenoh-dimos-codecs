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

use super::compressed_video::{AccessUnit, Codec};
use anyhow::{Result, anyhow, ensure};
use zenoh_gateway::{DecodedFrame, EncodedVideo, VideoEncoder, VideoFormat, VideoTarget};

/// Forwards already-encoded H.265 access units ([`AccessUnit`], from a compressed-video encoding's
/// [`decode`](zenoh_gateway::MessageEncoding::decode) on a `video-h265` channel) to the gateway's track.
///
/// One per encode session, so its remembered size and parameter sets follow one stream.
#[derive(Debug, Default)]
pub struct H265Passthrough {
    /// the newest size any sequence parameter set gave: frames between keyframes carry none
    size: Option<(u32, u32)>,
    /// the newest parameter sets seen, to put back in front of a keyframe that arrives without them
    parameter_sets: Option<Vec<u8>>,
}

impl VideoEncoder for H265Passthrough {
    fn format(&self) -> VideoFormat {
        VideoFormat::H265
    }

    fn encode(&mut self, frame: &DecodedFrame, _target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        let unit: &AccessUnit = frame.downcast().map_err(|error| anyhow!("the H.265 passthrough encoder takes access units: {error:#}"))?;
        ensure!(unit.codec == Codec::H265, "the H.265 passthrough encoder was handed {:?} data", unit.codec);
        if let Some(sets) = &unit.parameter_sets {
            self.parameter_sets = Some(sets.clone());
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
        let data = match (unit.keyframe, unit.parameter_sets.is_none(), &self.parameter_sets) {
            (true, true, Some(sets)) => [sets.as_slice(), unit.data.as_slice()].concat(),
            _ => unit.data.clone(),
        };
        let keyframe = unit.keyframe && (unit.parameter_sets.is_some() || self.parameter_sets.is_some());
        Ok(Some(EncodedVideo { data, width, height, keyframe }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::compressed_video::to_access_unit;
    use crate::utils::tests::fixture;

    /// A `DecodedFrame` holding `unit`, as a compressed-video encoding's `decode` returns on a `video-h265` channel.
    fn frame(unit: AccessUnit) -> DecodedFrame {
        DecodedFrame::data(unit)
    }

    #[test]
    fn forwards_a_keyframe_with_its_size_and_bytes_unchanged() {
        let data = fixture("compressed/pattern.h265");
        let unit = to_access_unit(&data, "h265").unwrap();
        let mut encoder = H265Passthrough::default();
        let encoded = encoder.encode(&frame(unit), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        assert_eq!((encoded.width, encoded.height), (320, 240), "the size comes from the SPS, not the target");
        assert!(encoded.keyframe);
        assert_eq!(encoded.data, to_access_unit(&data, "h265").unwrap().data, "the access unit goes out as it came in, start codes normalised");
    }


    #[test]
    fn a_predicted_frame_keeps_the_size_the_keyframe_gave() {
        let mut encoder = H265Passthrough::default();
        encoder.encode(&frame(to_access_unit(&fixture("compressed/pattern.h265"), "h265").unwrap()), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap();
        // a bare TRAIL_R slice: no keyframe, no parameter sets, no size of its own
        let predicted = to_access_unit(&[0, 0, 0, 1, 0x02, 0x01, 0xaa], "h265").unwrap();
        assert_eq!(predicted.size, None);
        let encoded = encoder.encode(&frame(predicted), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        assert_eq!((encoded.width, encoded.height), (320, 240));
        assert!(!encoded.keyframe, "a predicted frame is not a keyframe");
    }

    #[test]
    fn nothing_goes_out_before_a_parameter_set_says_how_big_the_picture_is() {
        let mut encoder = H265Passthrough::default();
        let predicted = to_access_unit(&[0, 0, 0, 1, 0x02, 0x01, 0xaa], "h265").unwrap();
        assert!(encoder.encode(&frame(predicted), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().is_none());
    }

    #[test]
    fn puts_remembered_parameter_sets_back_in_front_of_a_bare_keyframe() {
        let with_sets = fixture("compressed/pattern.h265");
        let bare = fixture("compressed/pattern_bare.h265");
        let mut encoder = H265Passthrough::default();
        encoder.encode(&frame(to_access_unit(&with_sets, "h265").unwrap()), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap();
        let unit = to_access_unit(&bare, "h265").unwrap();
        assert_eq!(unit.parameter_sets, None, "the bare fixture carries none");
        let encoded = encoder.encode(&frame(unit), &VideoTarget::new(64, 64, 100_000, 30.0)).unwrap().expect("a frame");
        let normalized = to_access_unit(&bare, "h265").unwrap().data;
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
        let h264 = to_access_unit(&fixture("compressed/pattern.h264"), "h264").unwrap();
        let error = encoder.encode(&frame(h264), &target).unwrap_err().to_string();
        assert!(error.contains("was handed H264 data"), "{error}");
    }
}
