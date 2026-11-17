//! Lossless depth on a data channel (16UC1 / 32FC1 / mono16 images, raw or 16-bit png / jxl): nearest-neighbor downscale (never interpolated), as zenoh-gateway
//! fields (zstd-compressed by the bridge by default), see README "Wire formats":
//! `version=2`, `encoding` ("16UC1", "32FC1" or "mono16"), `stride`, `width`, `height`, `sourceWidth`,
//! `sourceHeight`, `data` (width × height u16 or f32, row-major).

use crate::codecs::msgs::{Header, RawImage};
use crate::utils::compressed_image::{self, FileFormat};
use crate::utils::{jxl, png, raw_pixels};
use anyhow::{Context, Result, bail, ensure};
use zenoh_gateway::Fields;

/// Depth encodings, sent as the `encoding` field (see README "Wire formats").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DepthEncoding {
    /// 16UC1: u16, millimeters by convention
    U16 = 1,
    /// 32FC1: f32, meters by convention
    F32 = 2,
    /// mono16: u16 intensity (e.g. an IR camera); delivered losslessly like depth when asked for
    Mono16 = 3,
}

pub enum DepthValues {
    U16(Vec<u16>),
    F32(Vec<f32>),
}

pub struct Depth {
    /// the image's header, sent with it
    pub header: Header,
    pub width: u32,
    pub height: u32,
    pub encoding: DepthEncoding,
    pub values: DepthValues,
}

/// A raw 16UC1 / 32FC1 / mono16 image as lossless depth values.
pub fn from_raw(image: &RawImage) -> Result<Depth> {
    let (width, height) = (image.width, image.height);
    let pixel_count = width as usize * height as usize;
    match image.encoding.as_str() {
        "16UC1" | "mono16" => {
            let mut values = Vec::with_capacity(pixel_count);
            for y in 0..height {
                values.extend(raw_pixels::row(image, y, 2)?.as_chunks::<2>().0.iter().map(|s| raw_pixels::read_u16(s, image.big_endian)));
            }
            let encoding = if image.encoding == "mono16" { DepthEncoding::Mono16 } else { DepthEncoding::U16 };
            Ok(Depth { header: image.header.clone(), width, height, encoding, values: DepthValues::U16(values) })
        }
        "32FC1" => {
            let mut values = Vec::with_capacity(pixel_count);
            for y in 0..height {
                values.extend(raw_pixels::row(image, y, 4)?.as_chunks::<4>().0.iter().map(|&bytes| {
                    if image.big_endian { f32::from_be_bytes(bytes) } else { f32::from_le_bytes(bytes) }
                }));
            }
            Ok(Depth { header: image.header.clone(), width, height, encoding: DepthEncoding::F32, values: DepthValues::F32(values) })
        }
        other => bail!("image encoding {other:?} is not depth (supported: 16UC1 32FC1 mono16)"),
    }
}

/// A 16-bit single-channel png or jxl (also ROS `compressedDepth` png) as lossless u16 depth.
pub fn from_compressed(header: &Header, data: &[u8], format: &str) -> Result<Depth> {
    // compressed_depth_image_transport prefixes a 12-byte config header (format enum + 2 floats)
    let data = if format.contains("compressedDepth") {
        ensure!(!format.contains("32FC1"), "compressedDepth 32FC1 is quantized inverse depth, not lossless; unsupported");
        data.get(12..).context("compressedDepth payload shorter than its header")?
    } else {
        data
    };
    let (width, height, values) = match compressed_image::sniff_format(data, format)? {
        FileFormat::Png => png::to_u16(data)?,
        FileFormat::Jxl => jxl::to_u16(data)?,
        other => bail!("{other:?} can't carry lossless 16-bit depth (use png or jxl)"),
    };
    Ok(Depth { header: header.clone(), width, height, encoding: DepthEncoding::U16, values: DepthValues::U16(values) })
}

pub const VERSION: u8 = 2;

/// Integer downscale factor for a quality: 1 at quality 1, up to 8 at quality 0.
pub fn stride(quality: f64) -> u32 {
    let linear_scale = 1.0 / 8.0 + 7.0 / 8.0 * quality.clamp(0.0, 1.0);
    (1.0 / linear_scale).round().max(1.0) as u32
}

/// Fraction of full-quality pixels sent at `quality` (the allocator's size prior).
pub fn size_factor(quality: f64) -> f64 {
    1.0 / (stride(quality) as f64).powi(2)
}

/// The allocator's size prior: lossless zstd (the default compression) roughly halves depth; lower quality sends
/// 1/stride² of the pixels.
pub fn estimated_bytes(payload_bytes: usize, quality: f64) -> f64 {
    payload_bytes as f64 * 0.5 * size_factor(quality)
}

pub fn encode(depth: &Depth, quality: f64) -> Vec<u8> {
    let stride = stride(quality);
    let width = depth.width.div_ceil(stride);
    let height = depth.height.div_ceil(stride);
    fn sample<T: Copy>(values: &[T], source_width: u32, (width, height, stride): (u32, u32, u32)) -> Vec<T> {
        (0..height).flat_map(|y| (0..width).map(move |x| values[(y * stride) as usize * source_width as usize + (x * stride) as usize])).collect()
    }
    let encoding = match depth.encoding {
        DepthEncoding::U16 => "16UC1",
        DepthEncoding::F32 => "32FC1",
        DepthEncoding::Mono16 => "mono16",
    };
    let fields = depth.header.add_to(Fields::new())
        .scalar("version", VERSION)
        .text("encoding", encoding)
        .scalar("stride", stride)
        .scalar("width", width)
        .scalar("height", height)
        .scalar("sourceWidth", depth.width)
        .scalar("sourceHeight", depth.height);
    match &depth.values {
        DepthValues::U16(values) => fields.array("data", &sample(values, depth.width, (width, height, stride))),
        DepthValues::F32(values) => fields.array("data", &sample(values, depth.width, (width, height, stride))),
    }
    .build()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codecs::msgs::{dimos_lcm_image, ros2_compressed_image, ros2_image};
    use crate::utils::tests::fixture;

    #[test]
    fn strides() {
        assert_eq!(stride(1.0), 1);
        assert_eq!(stride(0.5), 2);
        assert_eq!(stride(0.0), 8);
    }

    #[test]
    fn full_quality_round_trips_and_downscale_is_nearest() {
        let values: Vec<u16> = (0..12u16).collect();
        let depth = Depth { header: Header::default(), width: 4, height: 3, encoding: DepthEncoding::U16, values: DepthValues::U16(values.clone()) };
        let full = zenoh_gateway::fields::parse(&encode(&depth, 1.0)).unwrap();
        assert_eq!((full["version"].values(), full["encoding"].text(), full["stride"].values()), (vec![2.0], Some("16UC1"), vec![1.0]));
        assert_eq!(full["data"].values(), values.iter().map(|&value| value as f64).collect::<Vec<_>>());
        let half = zenoh_gateway::fields::parse(&encode(&depth, 0.5)).unwrap();
        assert_eq!((half["width"].values(), half["height"].values(), half["sourceWidth"].values()), (vec![2.0], vec![2.0], vec![4.0]));
        assert_eq!(half["data"].values(), [0.0, 2.0, 8.0, 10.0], "every value is a source value, never a blend");
        let floats = Depth { header: Header::default(), width: 2, height: 1, encoding: DepthEncoding::F32, values: DepthValues::F32(vec![1.5, f32::NAN]) };
        let floats = zenoh_gateway::fields::parse(&encode(&floats, 1.0)).unwrap();
        assert_eq!((floats["encoding"].text(), floats["data"].dtype), (Some("32FC1"), zenoh_gateway::fields::Dtype::F32));
    }

    #[test]
    fn depth_is_exact() {
        let payload = fixture("ros2/depth_16UC1.cdr");
        let depth = from_raw(&ros2_image::parse(&payload).unwrap()).unwrap();
        let DepthValues::U16(values) = depth.values else { panic!("u16 expected") };
        assert!(values.iter().enumerate().all(|(i, &v)| v as usize == 1000 + i % 320 + 4 * (i / 320)));
        let payload = fixture("dimos/depth_32FC1.bin");
        let DepthValues::F32(values) = from_raw(&dimos_lcm_image::parse(&payload).unwrap()).unwrap().values else { panic!("f32 expected") };
        assert!(values.iter().enumerate().all(|(i, &v)| v == 0.5 + (i % 320) as f32 / 128.0 + (i / 320) as f32 / 64.0));
        let message_bytes = fixture("ros2/compressed_jxl_depth16.cdr");
        let message = ros2_compressed_image::parse(&message_bytes).unwrap();
        let DepthValues::U16(values) = from_compressed(&message.header, message.data, &message.format).unwrap().values else { panic!() };
        assert!(values.iter().enumerate().all(|(i, &v)| v as usize == 1000 + i % 320 + 4 * (i / 320)), "jxl depth is lossless");
        assert!(from_raw(&ros2_image::parse(&fixture("ros2/image_rgb8.cdr")).unwrap()).is_err());
        assert!(from_compressed(&message.header, message.data, "32FC1; compressedDepth png").is_err(), "quantized inverse depth is refused");
        assert!(from_compressed(&message.header, &message.data[..8], "16UC1; compressedDepth png").is_err(), "shorter than its 12-byte header");
        assert!(from_compressed(&message.header, &fixture("compressed/pattern.png"), "png").is_err(), "an 8-bit color png is not depth");
    }
}
