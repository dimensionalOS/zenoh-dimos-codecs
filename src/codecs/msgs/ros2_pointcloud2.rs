//! `sensor_msgs/PointCloud2` from ROS 2 (rmw_zenoh, CDR), thinned and int16-quantized on a data channel as zenoh-gateway fields.

use super::{PointCloud, PointField};
use crate::utils::{cdr::Cdr, pointcloud};
use anyhow::{Result, ensure};
use zenoh_gateway::{MessageEncoding, Channel, EncodeOptions, EncodingOutput, EncodingSample, Compress, DecodedFrame};

/// `ros2_pointcloud2`
pub struct Ros2PointCloud2;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<PointCloud<'_>> {
    let mut cdr = Cdr::new(payload)?;
    let header = cdr.header()?;
    let height = cdr.u32()?;
    let width = cdr.u32()?;
    let field_count = cdr.u32()? as usize;
    ensure!(field_count <= super::MAX_POINT_FIELDS, "PointCloud2 with {field_count} fields");
    let mut fields = Vec::with_capacity(field_count);
    for _ in 0..field_count {
        let name = cdr.string()?;
        let offset = cdr.u32()?;
        let datatype = cdr.u8()?;
        let count = cdr.u32()?;
        fields.push(PointField { name, offset, datatype, count });
    }
    let big_endian = cdr.u8()? != 0;
    let point_step = cdr.u32()?;
    let row_step = cdr.u32()?;
    let data = cdr.byte_sequence()?;
    Ok(PointCloud { header, height, width, fields, big_endian, point_step, row_step, data })
}

impl MessageEncoding for Ros2PointCloud2 {
    fn name(&self) -> &str {
        "ros2_pointcloud2"
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
        let payload = fixture("ros2/pointcloud_xyz.cdr");
        let cloud = parse(&payload).unwrap();
        assert_eq!((cloud.width, cloud.height, cloud.point_step), (20000, 1, 12));
        assert_eq!(cloud.fields[0], PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 });
        assert_eq!(cloud.data.len(), 20000 * 12);
    }

    #[test]
    fn garbage_is_an_error() {
        assert!(parse(&[0, 1, 0, 0, 1]).is_err());
    }
}
