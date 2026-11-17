//! `sensor_msgs/CompressedImage` from ROS 2 (rmw_zenoh, CDR) as video: a JPEG, PNG, WebP or JPEG XL file to a picture for the
//! video encoder; on the data channel, the file itself or converted (`encodeOptions.format`).

use super::{CompressedImage, Headed};
use crate::utils::compressed_image::ImageFile;
use crate::utils::{cdr::Cdr, compressed_image};
use anyhow::Result;
use serde_json::{Map, Value};
use zenoh_gateway::{Channel, DecodedFrame, EncodeOptions, EncodingOutput, EncodingSample, MessageEncoding};

/// `ros2_compressed_image`
pub struct Ros2CompressedImage;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<CompressedImage<'_>> {
    let mut cdr = Cdr::new(payload)?;
    let header = cdr.header()?;
    let format = cdr.string()?;
    let data = cdr.byte_sequence()?;
    Ok(CompressedImage { header, format, data })
}

impl MessageEncoding for Ros2CompressedImage {
    fn name(&self) -> &str {
        "ros2_compressed_image"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Video
    }

    /// Video channels: no options. Data: `{format}`, `"passthrough"` (default, the file as sent), `"jpeg"` or `"png"`.
    fn output_on(&self, channel: Channel, options: &Map<String, Value>) -> Result<EncodingOutput, String> {
        compressed_image::output_on(self.name(), channel, options, true)
    }

    /// the message's header, which the gateway sends with whatever this makes of it
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
        let message = parse(sample.payload)?;
        Ok(match channel {
            Channel::Data => DecodedFrame::data(Headed { value: ImageFile::new(message.data, &message.format)?, header: message.header }),
            _ => DecodedFrame::Video(message.header.add_to_video(compressed_image::to_video(message.data, &message.format)?)),
        })
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        let headed = frame.downcast::<Headed<ImageFile>>()?;
        compressed_image::with_header(headed.value.encode(options)?, &headed.header, options)
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        compressed_image::estimated_bytes(payload_bytes, options, true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("ros2/compressed_png.cdr");
        let image = parse(&payload).unwrap();
        assert!(image.format.contains("png"), "{}", image.format);
        assert_eq!(image.data, fixture("compressed/pattern.png").as_slice());
    }
}
