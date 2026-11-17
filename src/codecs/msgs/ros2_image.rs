//! `sensor_msgs/Image` from ROS 2 (rmw_zenoh, CDR) as video: raw pixels (rgb8, bgr8, mono16, ...) or a file in the data field, to a
//! picture for the video encoder; on the data channel, a JPEG or PNG of it (`encodeOptions.format`).

use super::{Headed, RawImage};
use crate::utils::compressed_image;
use crate::utils::{cdr::Cdr, raw_pixels};
use anyhow::Result;
use serde_json::{Map, Value};
use zenoh_gateway::{Channel, DecodedFrame, EncodeOptions, EncodingOutput, EncodingSample, MessageEncoding};

/// `ros2_image`
pub struct Ros2Image;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<RawImage<'_>> {
    let mut cdr = Cdr::new(payload)?;
    let header = cdr.header()?;
    let height = cdr.u32()?;
    let width = cdr.u32()?;
    let encoding = cdr.string()?;
    let big_endian = cdr.u8()? != 0;
    let step = cdr.u32()?;
    let data = cdr.byte_sequence()?;
    Ok(RawImage { header, width, height, encoding, big_endian, step, data })
}

impl MessageEncoding for Ros2Image {
    fn name(&self) -> &str {
        "ros2_image"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Video
    }

    /// Video channels: no options. Data: `{format}`, `"jpeg"` (default, at the allocator's quality) or `"png"`.
    fn output_on(&self, channel: Channel, options: &Map<String, Value>) -> Result<EncodingOutput, String> {
        compressed_image::output_on(self.name(), channel, options, false)
    }

    /// the message's header, which the gateway sends with whatever this makes of it
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
        let image = parse(sample.payload)?;
        Ok(match channel {
            Channel::Data => DecodedFrame::data(Headed { value: raw_pixels::to_rgb(&image)?, header: image.header }),
            _ => DecodedFrame::Video(image.header.add_to_video(raw_pixels::to_video(&image)?)),
        })
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        let headed = frame.downcast::<Headed<raw_pixels::Rgb8>>()?;
        compressed_image::with_header(compressed_image::encode_rgb(&headed.value, options)?, &headed.header, options)
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        compressed_image::raw_estimated_bytes(payload_bytes, options)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("ros2/image_rgb8.cdr");
        let image = parse(&payload).unwrap();
        assert_eq!((image.width, image.height, image.step), (320, 240, 960));
        assert_eq!(image.encoding, "rgb8");
        assert!(!image.big_endian);
        assert_eq!(image.data.len(), 320 * 240 * 3);
        assert_eq!(&image.data[..3], &[255, 0, 0]);
    }

    #[test]
    fn another_type_is_an_error() {
        assert!(parse(&fixture("ros2/compressed_png.cdr")).is_err(), "truncated");
    }
}
