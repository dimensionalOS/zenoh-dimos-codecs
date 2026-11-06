//! `foxglove_msgs/CompressedVideo` from ROS 2 (rmw_zenoh, CDR) as video: an H.264 or H.265 keyframe (Annex-B, with its
//! parameter sets) to a picture for the video encoder.

use super::CompressedVideo;
use crate::utils::{cdr::Cdr, compressed_video};
use anyhow::Result;
use zenoh_gateway::{Channel, DecodedFrame, EncodingOutput, EncodingSample, MessageEncoding};

/// `ros2_compressed_video`
pub struct Ros2CompressedVideo;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<CompressedVideo<'_>> {
    let mut cdr = Cdr::new(payload)?;
    // builtin_interfaces/Time timestamp, then frame_id
    let header = cdr.header()?;
    let data = cdr.byte_sequence()?;
    let format = cdr.string()?;
    Ok(CompressedVideo { header, data, format })
}

impl MessageEncoding for Ros2CompressedVideo {
    fn name(&self) -> &str {
        "ros2_compressed_video"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Video
    }

    /// the message's timestamp and frame id, which the gateway sends with each picture
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, _channel: Channel) -> Result<DecodedFrame> {
        let message = parse(sample.payload)?;
        Ok(DecodedFrame::Video(message.header.add_to_video(compressed_video::to_video(message.data, &message.format)?)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("ros2/compressed_video_h265.cdr");
        let video = parse(&payload).unwrap();
        assert_eq!((video.format.as_str(), video.data), ("h265", fixture("compressed/pattern.h265").as_slice()));
    }
}
