//! `sensor_msgs/PointCloud2` from dimos (LCM over zenoh), thinned and int16-quantized on a data channel as zenoh-gateway fields.

use super::{PointCloud, PointField};
use crate::utils::{lcm::Lcm, pointcloud};
use anyhow::{Result, ensure};
use zenoh_gateway::{MessageEncoding, Channel, EncodeOptions, EncodingOutput, EncodingSample, Compress, DecodedFrame};

/// The LCM fingerprint of dimos-lcm's type, as `lcm-gen` computes it.
const LCM_POINT_CLOUD2: [u8; 8] = [0xf5, 0xeb, 0x3d, 0xa1, 0xc2, 0x85, 0x31, 0x75];

/// `dimos_lcm_pointcloud2`
pub struct DimosLcmPointCloud2;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<PointCloud<'_>> {
    let mut lcm = Lcm::new(payload, LCM_POINT_CLOUD2, "sensor_msgs.PointCloud2")?;
    let field_count = lcm.length()?;
    ensure!(field_count <= super::MAX_POINT_FIELDS, "PointCloud2 with {field_count} fields");
    let data_length = lcm.length()?;
    let header = lcm.header()?;
    let height = lcm.i32()? as u32;
    let width = lcm.i32()? as u32;
    let mut fields = Vec::with_capacity(field_count);
    for _ in 0..field_count {
        let name = lcm.string()?;
        let offset = lcm.i32()? as u32;
        let datatype = lcm.u8()?;
        let count = lcm.i32()? as u32;
        fields.push(PointField { name, offset, datatype, count });
    }
    let big_endian = lcm.u8()? != 0;
    let point_step = lcm.i32()? as u32;
    let row_step = lcm.i32()? as u32;
    let data = lcm.take(data_length)?;
    Ok(PointCloud { header, height, width, fields, big_endian, point_step, row_step, data })
}

impl MessageEncoding for DimosLcmPointCloud2 {
    fn name(&self) -> &str {
        "dimos_lcm_pointcloud2"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Fields
    }

    fn default_compress(&self) -> Compress {
        Compress::Zstd
    }

    /// the message's header, which the gateway sends with whatever this makes of it
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, _channel: Channel) -> Result<DecodedFrame> {
        Ok(DecodedFrame::data(pointcloud::read_points(&parse(sample.payload)?)?))
    }

    fn encode(&self, frame: &DecodedFrame, options: &EncodeOptions) -> Result<Vec<u8>> {
        Ok(pointcloud::encode_points(frame.downcast::<pointcloud::Points>()?, options.quality))
    }

    fn estimated_bytes(&self, payload_bytes: usize, options: &EncodeOptions) -> f64 {
        pointcloud::estimated_bytes(payload_bytes, options.quality)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn parses() {
        let payload = fixture("dimos/pointcloud_xyzi.bin");
        let cloud = parse(&payload).unwrap();
        assert_eq!((cloud.width, cloud.height, cloud.point_step), (20000, 1, 16));
        assert_eq!(cloud.fields[0], PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 });
        assert_eq!(cloud.data.len(), 20000 * 16);
    }
}
