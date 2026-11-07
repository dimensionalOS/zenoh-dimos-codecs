//! The stateful side of compressed video: a [`StreamDecoder`] that keeps one H.265 decoder per stream, so predicted
//! frames decode and the gateway can re-encode all of them.
//!
//! This is the path a browser with no HEVC decoder takes (Firefox, which has no H.265 over WebRTC). Such a browser
//! cannot be sent the publisher's bitstream, so the gateway has to decode it and encode it again — and
//! [`compressed_video::to_video`](super::compressed_video::to_video) decodes each sample on its own, which only a
//! keyframe can be. On this camera that is one frame in thirty.
//!
//! A decoder that keeps its state fixes that but changes what it needs: every sample of the stream, in arrival order,
//! with nothing skipped. The gateway arranges that for a `StreamDecoder` — it stops thinning the stream, decodes one
//! sample at a time, and shares the frames with nobody, since another subscription's decoder sits at another point in
//! the stream. When samples are lost anyway, it calls [`resync`](StreamDecoder::resync) and this skips to the next
//! keyframe rather than showing a picture built on frames it never saw.

use super::compressed_video::{self, Codec};
use super::h265;
use crate::codecs::msgs::CompressedVideo;
use anyhow::{Result, bail};
use zenoh_gateway::{DecodedFrame, EncodingSample, StreamDecoder};

/// How a compressed-video encoding reads its own wire format (`ros2_compressed_video`'s CDR,
/// `dimos_lcm_compressed_video`'s LCM), so one decoder serves both.
pub type Parse = for<'a> fn(&'a [u8]) -> Result<CompressedVideo<'a>>;

/// Keeps one H.265 decoder for a stream of `foxglove_msgs/CompressedVideo`, so its predicted frames decode.
///
/// The decoder is made on the first H.265 sample rather than up front: the gateway asks for one of these before any
/// sample has arrived, so the format is not known yet, and a stream that turns out to be H.264 should not have started
/// libde265 and its threads for nothing.
pub struct CompressedVideoStream {
    parse: Parse,
    h265: Option<h265::Stream>,
    /// the newest parameter sets, to put back in front of a keyframe that arrives without them
    parameter_sets: Option<Vec<u8>>,
}

impl CompressedVideoStream {
    /// A decoder for an encoding that reads its messages with `parse`.
    pub fn new(parse: Parse) -> Self {
        CompressedVideoStream { parse, h265: None, parameter_sets: None }
    }
}

impl StreamDecoder for CompressedVideoStream {
    fn decode(&mut self, sample: &EncodingSample<'_>) -> Result<Option<DecodedFrame>> {
        let message = (self.parse)(sample.payload)?;
        match Codec::from_format(&message.format)? {
            Codec::H265 => {}
            // openh264's decoder would do the same job, but nothing here needs it yet and an untested path is worse
            // than a clear refusal: H.264 keyframes still play through the per-sample decode.
            Codec::H264 => bail!("this stream is H.264, which has no stateful decoder here (its keyframes still play one by one)"),
        }
        let unit = compressed_video::to_access_unit(message.data, &message.format)?;
        let (data, keyframe) = compressed_video::decodable(&unit, &mut self.parameter_sets);
        let stream = match &mut self.h265 {
            Some(stream) => stream,
            none => none.insert(h265::Stream::new()?),
        };
        Ok(stream.push(&data, keyframe)?.map(|picture| DecodedFrame::Video(message.header.add_to_video(picture))))
    }

    fn resync(&mut self) {
        if let Some(stream) = &mut self.h265 {
            stream.resync();
        }
        // the parameter sets are kept: they describe the stream, not the frames that were lost
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codecs::msgs::ros2_compressed_video;
    use crate::utils::compressed_video::{Codec, nal_units};
    use crate::utils::tests::fixture;

    /// `foxglove_msgs/CompressedVideo` as CDR, so the decoder is fed what the gateway would feed it.
    fn cdr(data: &[u8], format: &str) -> Vec<u8> {
        let mut body: Vec<u8> = Vec::with_capacity(data.len() + 64);
        let align = |body: &mut Vec<u8>| while body.len() % 4 != 0 { body.push(0) };
        let string = |body: &mut Vec<u8>, text: &str| {
            align(body);
            body.extend_from_slice(&(text.len() as u32 + 1).to_le_bytes());
            body.extend_from_slice(text.as_bytes());
            body.push(0);
        };
        body.extend_from_slice(&1_700_000_000i32.to_le_bytes());
        body.extend_from_slice(&0u32.to_le_bytes());
        string(&mut body, "camera_color_optical_frame");
        align(&mut body);
        body.extend_from_slice(&(data.len() as u32).to_le_bytes());
        body.extend_from_slice(data);
        string(&mut body, format);
        [&[0x00, 0x01, 0x00, 0x00][..], &body].concat()
    }

    fn decode(decoder: &mut CompressedVideoStream, data: &[u8], format: &str) -> Result<Option<DecodedFrame>> {
        let payload = cdr(data, format);
        let encoding = zenoh_gateway::zenoh::bytes::Encoding::default();
        decoder.decode(&EncodingSample::new("camera/front", &payload, &encoding))
    }

    /// The access units of the robot's capture: its parameter sets are not in band, so only the keyframe fixture has
    /// them, and the publisher is what puts them in front — which is what `decodable` stands in for.
    fn capture() -> (Vec<u8>, Vec<Vec<u8>>) {
        let predicted = nal_units(Codec::H265, &fixture("compressed/m20_front_2s.h265"))
            .into_iter()
            .filter(|(kind, _)| *kind == 1)
            .map(|(_, unit)| [&[0, 0, 0, 1][..], unit].concat())
            .collect();
        (fixture("compressed/m20_front_keyframe.h265"), predicted)
    }

    #[test]
    fn decodes_the_predicted_frames_a_per_sample_decode_cannot() {
        if h265::Stream::new().is_err() {
            eprintln!("skipping: libde265 is not installed");
            return;
        }
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        let first = decode(&mut decoder, &keyframe, "h265").unwrap().expect("the keyframe decodes");
        let DecodedFrame::Video(image) = first else { panic!("a picture") };
        assert_eq!((image.width(), image.height()), (1280, 720));
        // its message's header rides along, as it does on the per-sample path
        assert_eq!(image.message_header().map(|header| header.frame_id.clone()), Some("camera_color_optical_frame".to_owned()));
        let mut pictures = 1;
        for unit in &predicted {
            if decode(&mut decoder, unit, "h265").unwrap().is_some() {
                pictures += 1;
            }
        }
        assert_eq!(pictures, 1 + predicted.len(), "every predicted frame decoded, not just the keyframe");
    }

    #[test]
    fn shows_nothing_until_a_keyframe_and_again_after_a_gap() {
        if h265::Stream::new().is_err() {
            return;
        }
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        // before any keyframe there is nothing to predict from
        assert!(decode(&mut decoder, &predicted[0], "h265").unwrap().is_none());
        assert!(decode(&mut decoder, &keyframe, "h265").unwrap().is_some());
        assert!(decode(&mut decoder, &predicted[0], "h265").unwrap().is_some());
        // after a gap it waits for the next keyframe rather than showing a picture built on frames it never saw
        decoder.resync();
        assert!(decode(&mut decoder, &predicted[1], "h265").unwrap().is_none());
        assert!(decode(&mut decoder, &keyframe, "h265").unwrap().is_some());
    }

    #[test]
    fn says_why_it_cannot_take_h264() {
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        let error = decode(&mut decoder, &fixture("compressed/pattern.h264"), "h264").unwrap_err().to_string();
        assert!(error.contains("no stateful decoder here"), "{error}");
    }
}
