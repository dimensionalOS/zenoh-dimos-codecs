//! `sensor_msgs/Image` depth (16UC1, 32FC1, mono16) from dimos (LCM over zenoh), lossless on a data channel as zenoh-gateway fields.

use super::dimos_lcm_image;
use crate::utils::depth;
use anyhow::Result;
use zenoh_gateway::{MessageEncoding, Channel, EncodeOptions, EncodingOutput, EncodingSample, Compress, DecodedFrame};

/// `dimos_lcm_depth`
pub struct DimosLcmDepth;

impl MessageEncoding for DimosLcmDepth {
    fn name(&self) -> &str {
        "dimos_lcm_depth"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    /// the message's header, which the gateway sends with whatever this makes of it
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        dimos_lcm_image::parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, _channel: Channel) -> Result<DecodedFrame> {
        Ok(DecodedFrame::data(depth::from_raw(&dimos_lcm_image::parse(sample.payload)?)?))
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        Ok(depth::encode(frame.downcast::<depth::Depth>()?, options.quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        depth::estimated_bytes(payload_bytes, options.quality)
    }
}
