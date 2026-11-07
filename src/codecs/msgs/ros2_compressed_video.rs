//! `foxglove_msgs/CompressedVideo` from ROS 2 (rmw_zenoh, CDR) as video: an H.264 or H.265 keyframe (Annex-B, with its
//! parameter sets) to a picture for the video encoder.

use super::CompressedVideo;
use crate::utils::{cdr::Cdr, compressed_video};
use anyhow::Result;
use crate::utils::video_passthrough::H265Passthrough;
use crate::utils::video_stream::CompressedVideoStream;
use zenoh_gateway::{Channel, DecodedFrame, EncodingOutput, EncodingSample, MessageEncoding, StreamDecoder, VideoEncoder, VideoFormat};

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

    fn decode(&self, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
        let message = parse(sample.payload)?;
        // on a passthrough channel the access unit goes to the browser as it is, predicted frames and all
        if compressed_video::passes_through(channel) {
            return Ok(DecodedFrame::data(compressed_video::to_access_unit(message.data, &message.format)?));
        }
        Ok(DecodedFrame::Video(message.header.add_to_video(compressed_video::to_video(message.data, &message.format)?)))
    }

    /// H.265 straight to the browser, which decodes it in hardware: no decode and re-encode here, so predicted frames
    /// play too (`video-h265`; only Chrome and Safari offer it, see the client's `playableChannel`).
    fn video_encoder(&self, format: VideoFormat) -> Option<Box<dyn VideoEncoder>> {
        (format == VideoFormat::H265).then(|| Box::new(H265Passthrough::default()) as Box<dyn VideoEncoder>)
    }

    /// Anywhere the stream is decoded rather than passed through, one decoder keeps its state so predicted frames
    /// decode too — without it only keyframes play, about one a second. A passthrough channel needs none: the browser's
    /// own decoder keeps the state there.
    fn stream_decoder(&self, channel: Channel) -> Option<Box<dyn StreamDecoder>> {
        let decoded_here = matches!(channel, Channel::Video(format) if !format.passthrough_only());
        decoded_here.then(|| Box::new(CompressedVideoStream::new(parse)) as Box<dyn StreamDecoder>)
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
