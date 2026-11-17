//! `sensor_msgs/CompressedImage` depth (16-bit PNG or JPEG XL, also ROS `compressedDepth`) from ROS 2 (rmw_zenoh, CDR), lossless on a
//! data channel as zenoh-gateway fields.

use super::ros2_compressed_image;
use crate::utils::depth;
use anyhow::Result;
use zenoh_gateway::{MessageEncoding, Channel, EncodeOptions, EncodingOutput, EncodingSample, Compress, DecodedFrame};

/// `ros2_compressed_depth`
pub struct Ros2CompressedDepth;

impl MessageEncoding for Ros2CompressedDepth {
    fn name(&self) -> &str {
        "ros2_compressed_depth"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    /// the message's header, which the gateway sends with whatever this makes of it
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        ros2_compressed_image::parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, _channel: Channel) -> Result<DecodedFrame> {
        let message = ros2_compressed_image::parse(sample.payload)?;
        Ok(DecodedFrame::data(depth::from_compressed(&message.header, message.data, &message.format)?))
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        Ok(depth::encode(frame.downcast::<depth::Depth>()?, options.quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        depth::estimated_bytes(payload_bytes, options.quality)
    }
}
