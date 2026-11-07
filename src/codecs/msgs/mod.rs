//! One codec per message type: each file parses its message (ROS 2: CDR, dimos: LCM) and implements
//! [`MessageEncoding`](zenoh_gateway::MessageEncoding). Only the fields a codec needs are kept, and its header (stamp, frame_id), which goes to the page.

pub mod dimos_lcm_compressed_depth;
pub mod dimos_lcm_compressed_image;
pub mod dimos_lcm_compressed_video;
pub mod dimos_lcm_depth;
pub mod dimos_lcm_image;
pub mod dimos_lcm_pointcloud2;
pub mod dimos_lcm_raw_audio;
pub mod ros2_compressed_depth;
pub mod ros2_compressed_image;
pub mod ros2_compressed_video;
pub mod ros2_depth;
pub mod ros2_image;
pub mod ros2_pointcloud2;
pub mod ros2_raw_audio;

pub use dimos_lcm_compressed_depth::DimosLcmCompressedDepth;
pub use dimos_lcm_compressed_image::DimosLcmCompressedImage;
pub use dimos_lcm_compressed_video::DimosLcmCompressedVideo;
pub use dimos_lcm_depth::DimosLcmDepth;
pub use dimos_lcm_image::DimosLcmImage;
pub use dimos_lcm_pointcloud2::DimosLcmPointCloud2;
pub use dimos_lcm_raw_audio::DimosLcmRawAudio;
pub use ros2_compressed_depth::Ros2CompressedDepth;
pub use ros2_compressed_image::Ros2CompressedImage;
pub use ros2_compressed_video::Ros2CompressedVideo;
pub use ros2_depth::Ros2Depth;
pub use ros2_image::Ros2Image;
pub use ros2_pointcloud2::Ros2PointCloud2;
pub use ros2_raw_audio::Ros2RawAudio;

/// The most PointCloud2 fields a parser takes, a bound on a length read off the wire before allocating (real clouds
/// have a handful: x, y, z, intensity, ring, time, ...).
pub(crate) const MAX_POINT_FIELDS: usize = 1024;

// The message shapes both protocols share.

/// `sensor_msgs/Image`
#[derive(Debug)]
pub struct RawImage<'a> {
    pub header: Header,
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub big_endian: bool,
    pub step: u32,
    pub data: &'a [u8],
}

/// A message's std_msgs/Header: what an encoding passes on to the page (the encoded output has no other header)
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Header {
    /// seconds since the epoch (`sec` + `nanosec` are exact)
    pub stamp: f64,
    pub sec: i32,
    pub nanosec: u32,
    pub frame_id: String,
    /// dimos LCM's; ROS 2 has none
    pub seq: Option<u32>,
}

impl Header {
    pub fn new(sec: i32, nanosec: u32, frame_id: String, seq: Option<u32>) -> Self {
        Header { stamp: sec as f64 + nanosec as f64 * 1e-9, sec, nanosec, frame_id, seq }
    }
    /// what the gateway sends with every message ([`MessageEncoding::header`](zenoh_gateway::MessageEncoding::header))
    pub fn to_gateway(&self) -> zenoh_gateway::MessageHeader {
        zenoh_gateway::MessageHeader { sec: self.sec, nanosec: self.nanosec, frame_id: self.frame_id.clone(), seq: self.seq }
    }
    /// `stamp` (s) and `frameId` added to an encoding's fields
    pub fn add_to(&self, fields: zenoh_gateway::Fields) -> zenoh_gateway::Fields {
        fields.scalar("stamp", self.stamp).text("frameId", &self.frame_id)
    }
    /// the picture with this header, which the gateway sends with its frame's metadata (`msg.video.stamp`, `msg.video.frameId`)
    pub fn add_to_video(&self, image: zenoh_gateway::VideoImage) -> zenoh_gateway::VideoImage {
        image.with_message_header(self.to_gateway())
    }
}

/// A data-channel decoded form with its message's header (for `encodeOptions.header`).
pub struct Headed<T> {
    pub header: Header,
    pub value: T,
}

/// `sensor_msgs/CompressedImage`
#[derive(Debug)]
pub struct CompressedImage<'a> {
    pub header: Header,
    pub format: String,
    pub data: &'a [u8],
}

/// `foxglove_msgs/CompressedVideo`
#[derive(Debug)]
pub struct CompressedVideo<'a> {
    /// its `timestamp` and `frame_id` (no seq)
    pub header: Header,
    /// one access unit, Annex-B
    pub data: &'a [u8],
    /// `h264`, `h265`, ...
    pub format: String,
}

/// A `CompressedVideo` message type's stream decoder: its key's access units in order (zenoh-gateway's stream decoding),
/// each picture with its message's header.
pub struct CompressedVideoStream {
    parse: for<'a> fn(&'a [u8]) -> anyhow::Result<CompressedVideo<'a>>,
    pictures: crate::utils::compressed_video::Pictures,
}

impl CompressedVideoStream {
    pub fn new(parse: for<'a> fn(&'a [u8]) -> anyhow::Result<CompressedVideo<'a>>) -> Self {
        CompressedVideoStream { parse, pictures: Default::default() }
    }

    /// Whether decoding can start at this message: it parses and holds a keyframe.
    pub fn starts(parse: for<'a> fn(&'a [u8]) -> anyhow::Result<CompressedVideo<'a>>, payload: &[u8]) -> bool {
        use crate::utils::compressed_video::{Codec, is_keyframe};
        parse(payload).is_ok_and(|message| Codec::from_format(&message.format).is_ok_and(|codec| is_keyframe(codec, message.data)))
    }
}

impl zenoh_gateway::StreamDecoder for CompressedVideoStream {
    fn feed(&mut self, sample: &zenoh_gateway::EncodingSample<'_>, _channel: zenoh_gateway::Channel) -> anyhow::Result<Option<zenoh_gateway::DecodedFrame>> {
        let message = (self.parse)(sample.payload)?;
        let picture = self.pictures.feed(message.data, &message.format)?;
        Ok(picture.map(|picture| zenoh_gateway::DecodedFrame::Video(message.header.add_to_video(picture))))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    /// sensor_msgs/PointField datatype: 1 INT8 .. 8 FLOAT64
    pub datatype: u8,
    pub count: u32,
}

/// `sensor_msgs/PointCloud2`
#[derive(Debug)]
pub struct PointCloud<'a> {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub big_endian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: &'a [u8],
}

/// `foxglove_msgs/RawAudio`
#[derive(Debug)]
pub struct RawAudio<'a> {
    /// its `timestamp` (foxglove's RawAudio has no frame id or seq)
    pub header: Header,
    /// interleaved little-endian samples
    pub data: &'a [u8],
    /// `pcm-s16`
    pub format: String,
    pub sample_rate: u32,
    pub channels: u32,
}
