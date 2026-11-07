//! `foxglove_msgs/CompressedVideo` from dimos (LCM over zenoh) as video: H.264 or H.265 access units (Annex-B), decoded
//! in order from the first keyframe on, to pictures for the video encoder.

use super::CompressedVideo;
use crate::utils::{compressed_video, lcm::Lcm};
use anyhow::Result;
use zenoh_gateway::{Channel, DecodedFrame, EncodingOutput, EncodingSample, MessageEncoding};

/// The LCM fingerprint of dimos-lcm's type, as `lcm-gen` computes it.
const LCM_COMPRESSED_VIDEO: [u8; 8] = [0x22, 0x17, 0xb4, 0x6e, 0xef, 0xbf, 0x7b, 0x05];

/// `dimos_lcm_compressed_video`
pub struct DimosLcmCompressedVideo;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<CompressedVideo<'_>> {
    let mut lcm = Lcm::new(payload, LCM_COMPRESSED_VIDEO, "foxglove_msgs.CompressedVideo")?;
    let data_length = lcm.length()?;
    // builtin_interfaces.Time timestamp
    let (sec, nsec) = (lcm.i32()?, lcm.i32()?);
    let frame_id = lcm.string()?;
    let data = lcm.take(data_length)?;
    let format = lcm.string()?;
    Ok(CompressedVideo { header: super::Header::new(sec, nsec as u32, frame_id, None), data, format })
}

impl MessageEncoding for DimosLcmCompressedVideo {
    fn name(&self) -> &str {
        "dimos_lcm_compressed_video"
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

    /// a decoder per key, fed every access unit in order: predicted frames play, from the first keyframe on
    fn stream_decoder(&self) -> Option<Box<dyn zenoh_gateway::StreamDecoder>> {
        Some(Box::new(super::CompressedVideoStream::new(parse)))
    }

    fn starts_stream(&self, sample: &EncodingSample<'_>) -> bool {
        super::CompressedVideoStream::starts(parse, sample.payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("dimos/compressed_video_h265.bin");
        let video = parse(&payload).unwrap();
        assert_eq!((video.format.as_str(), video.data), ("h265", fixture("compressed/pattern.h265").as_slice()));
    }

    #[test]
    fn h264_keyframes_decode() {
        let payload = fixture("dimos/compressed_video_h264.bin");
        let encoding = zenoh_gateway::zenoh::bytes::Encoding::default();
        let DecodedFrame::Video(image) = DimosLcmCompressedVideo.decode(&EncodingSample::new("k", &payload, &encoding), Channel::Video(zenoh_gateway::VideoFormat::H264)).unwrap() else { panic!("video expected") };
        crate::utils::compressed_video::tests::assert_like_pattern(&image, "dimos h264");
    }
}
