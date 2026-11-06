//! Compressed video (`foxglove_msgs/CompressedVideo` data): one Annex-B access unit of H.264 or H.265 to a picture for
//! the video encoder. Each sample is decoded on its own (the gateway decodes only the samples it sends), so a sample must
//! be a keyframe carrying its parameter sets: a stream with predicted frames, or parameter sets sent once, can't play.

use crate::utils::{h264, h265};
use anyhow::{Result, bail, ensure};
use zenoh_gateway::VideoImage;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codec {
    H264,
    H265,
}

impl Codec {
    /// From the message's `format` (foxglove: `h264`, `h265`; `hevc` and `avc` too).
    pub fn from_format(format: &str) -> Result<Self> {
        Ok(match format.trim().to_ascii_lowercase().as_str() {
            "h264" | "avc" => Codec::H264,
            "h265" | "hevc" => Codec::H265,
            other => bail!("compressed video format {other:?} isn't h264 or h265"),
        })
    }

    /// A NAL unit's type, from its first byte.
    fn nal_type(self, first: u8) -> u8 {
        match self {
            Codec::H264 => first & 0x1f,
            Codec::H265 => (first >> 1) & 0x3f,
        }
    }

    /// (keyframe slice types, parameter sets a keyframe needs, their names)
    fn keyframe_needs(self) -> (std::ops::RangeInclusive<u8>, &'static [(u8, &'static str)]) {
        match self {
            // IDR
            Codec::H264 => (5..=5, &[(7, "SPS"), (8, "PPS")]),
            // IRAP: BLA, IDR, CRA
            Codec::H265 => (16..=21, &[(32, "VPS"), (33, "SPS"), (34, "PPS")]),
        }
    }
}

/// The types of the NAL units in an Annex-B access unit, in order.
fn nal_types(codec: Codec, data: &[u8]) -> Vec<u8> {
    let mut types = Vec::new();
    let mut at = 0;
    while let Some(found) = data[at..].windows(3).position(|window| window == [0, 0, 1]) {
        at += found + 3;
        if let Some(&first) = data.get(at) {
            types.push(codec.nal_type(first));
        }
    }
    types
}

/// Fails, saying why, unless `data` is a keyframe with its parameter sets.
fn check_decodable_alone(codec: Codec, data: &[u8]) -> Result<()> {
    let types = nal_types(codec, data);
    ensure!(!types.is_empty(), "{codec:?} data has no Annex-B start code (00 00 01)");
    let (keyframes, parameter_sets) = codec.keyframe_needs();
    ensure!(types.iter().any(|kind| keyframes.contains(kind)), "{codec:?} sample isn't a keyframe (NAL types {types:?}): each sample is decoded on its own, so the publisher must send keyframes only");
    let missing: Vec<&str> = parameter_sets.iter().filter(|(kind, _)| !types.contains(kind)).map(|(_, name)| *name).collect();
    ensure!(missing.is_empty(), "{codec:?} keyframe has no {} (NAL types {types:?}): each sample is decoded on its own, so the publisher must put the parameter sets in front of every keyframe", missing.join("/"));
    Ok(())
}

/// The picture in one access unit of `format` (`h264` or `h265`).
pub fn to_video(data: &[u8], format: &str) -> Result<VideoImage> {
    let codec = Codec::from_format(format)?;
    check_decodable_alone(codec, data)?;
    match codec {
        Codec::H264 => h264::decode(data),
        Codec::H265 => h265::decode(data),
    }
}

/// An I420 picture from 8-bit 4:2:0 planes with row strides, cropped to even sizes (what I420 takes).
pub(crate) fn i420_from_planes(width: usize, height: usize, planes: [(&[u8], usize); 3]) -> Result<VideoImage> {
    let (even_width, even_height) = (width & !1, height & !1);
    ensure!(even_width > 0 && even_height > 0, "empty picture ({width}x{height})");
    let mut data = Vec::with_capacity(even_width * even_height * 3 / 2);
    for (index, (plane, stride)) in planes.into_iter().enumerate() {
        let (columns, rows) = if index == 0 { (even_width, even_height) } else { (even_width / 2, even_height / 2) };
        ensure!(stride >= columns && plane.len() >= stride * (rows - 1) + columns, "plane {index} too small for {width}x{height}");
        for row in 0..rows {
            data.extend_from_slice(&plane[row * stride..row * stride + columns]);
        }
    }
    VideoImage::i420(even_width as u32, even_height as u32, data)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    /// Fails unless `image` is the 320x240 pattern, to within a codec's loss (mean difference per sample).
    pub(crate) fn assert_like_pattern(image: &VideoImage, what: &str) {
        let rgb = crate::utils::png::to_rgb(&fixture("compressed/pattern.png")).unwrap();
        let expected = VideoImage::rgb8(rgb.width, rgb.height, rgb.pixels).unwrap().to_i420(320, 240).unwrap();
        assert_eq!((image.width(), image.height()), (320, 240), "{what}");
        let difference = image.data().iter().zip(expected.data()).map(|(a, b)| a.abs_diff(*b) as f64).sum::<f64>() / expected.data().len() as f64;
        assert!(difference < 3.0, "{what}: mean difference {difference}");
    }

    #[test]
    fn a_sample_must_be_a_keyframe_with_its_parameter_sets() {
        assert!(check_decodable_alone(Codec::H265, &fixture("compressed/pattern.h265")).is_ok());
        assert!(check_decodable_alone(Codec::H264, &fixture("compressed/pattern.h264")).is_ok());
        let bare = check_decodable_alone(Codec::H265, &fixture("compressed/pattern_bare.h265")).unwrap_err().to_string();
        assert!(bare.contains("no VPS/SPS/PPS"), "{bare}");
        // an H.265 trailing picture (TRAIL_R, type 1) is predicted
        let predicted = check_decodable_alone(Codec::H265, &[0, 0, 0, 1, 0x02, 0x01, 0xaa]).unwrap_err().to_string();
        assert!(predicted.contains("isn't a keyframe"), "{predicted}");
        assert!(check_decodable_alone(Codec::H264, b"no start code").is_err());
        assert!(Codec::from_format("vp9").is_err());
        assert_eq!(Codec::from_format(" HEVC ").unwrap(), Codec::H265);
    }

    #[test]
    fn planes_with_strides_crop_to_even_sizes() {
        // 3x3 with stride 4 (chroma 2x2 with stride 2): crops to 2x2
        let luma = [1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 0];
        let image = i420_from_planes(3, 3, [(&luma, 4), (&[10, 11, 12, 13], 2), (&[20, 21, 22, 23], 2)]).unwrap();
        assert_eq!((image.width(), image.height(), image.data()), (2, 2, &[1, 2, 4, 5, 10, 20][..]));
    }
}
