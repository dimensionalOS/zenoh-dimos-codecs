//! `foxglove_msgs/CompressedVideo` from dimos (LCM over zenoh) as video: an H.264 or H.265 keyframe (Annex-B, with its
//! parameter sets) to a picture for the video encoder.

use super::CompressedVideo;
use crate::utils::{lcm::Lcm, video};
use anyhow::Result;
use zenoh_gateway::{Channel, DecodedFrame, EncodingOutput, EncodingSample, MessageEncoding, StreamDecoder, VideoEncoder, VideoFormat};

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

    fn decode(&self, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
        video::decode(parse, sample, channel)
    }

    /// A track whose format the publisher already sends is forwarded, so predicted frames play and nothing is
    /// transcoded; any other track is decoded and encoded again. See [`video`](crate::utils::video).
    fn video_encoder(&self, format: VideoFormat) -> Option<Box<dyn VideoEncoder>> {
        video::video_encoder(format)
    }

    /// One decoder per stream, so predicted frames play (without it only keyframes do, about one a second) and so the
    /// gateway stops thinning a stream whose frames predict from each other.
    fn stream_decoder(&self, channel: Channel) -> Option<Box<dyn StreamDecoder>> {
        video::stream_decoder(parse, channel)
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

    /// The per-sample path decodes an H.264 keyframe when the track is **not** H.264 — on an H.264 track the same
    /// sample is forwarded instead, which is the pair rule and is what makes the decode here the thing under test.
    #[test]
    fn h264_keyframes_decode_for_a_track_they_cannot_be_forwarded_to() {
        let payload = fixture("dimos/compressed_video_h264.bin");
        let encoding = zenoh_gateway::zenoh::bytes::Encoding::default();
        let sample = EncodingSample::new("k", &payload, &encoding);
        let DecodedFrame::Video(image) = DimosLcmCompressedVideo.decode(&sample, Channel::Video(VideoFormat::Vp8)).unwrap() else { panic!("video expected") };
        crate::utils::compressed_video::tests::assert_like_pattern(&image, "dimos h264");
        // and on its own format it is handed over whole, not decoded
        let forwarded = DimosLcmCompressedVideo.decode(&sample, Channel::Video(VideoFormat::H264)).unwrap();
        assert!(matches!(forwarded, DecodedFrame::Data(_)), "an H.264 source on an H.264 track is forwarded");
    }
}
