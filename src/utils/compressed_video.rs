//! Compressed video (`foxglove_msgs/CompressedVideo` data): Annex-B access units of H.264 or H.265 to pictures for the
//! video encoder. [`Pictures`] decodes a stream in order (predicted frames too, from its first keyframe on); [`to_video`]
//! decodes one sample alone, so it must be a keyframe carrying its parameter sets.

use crate::utils::{h264, h265};
use anyhow::{Context, Result, bail, ensure};
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

/// Whether `data` holds a keyframe (a picture that decodes without any before it).
pub fn is_keyframe(codec: Codec, data: &[u8]) -> bool {
    let (keyframes, _) = codec.keyframe_needs();
    nal_types(codec, data).iter().any(|kind| keyframes.contains(kind))
}

enum Decoder {
    H264(h264::Stream),
    H265(h265::Stream),
}

/// One stream's pictures: each access unit is decoded with the state the ones before it left. Predicted frames before
/// the first keyframe, and after a broken sample (a decoding error, or a lost reference), show nothing until the next
/// keyframe.
#[derive(Default)]
pub struct Pictures {
    decoder: Option<(Codec, Decoder)>,
    /// a keyframe decoded and nothing broke since
    synced: bool,
}

impl Pictures {
    /// The picture in the next access unit of `format` (`h264` or `h265`), `None` while it has none to show; an error
    /// once for a broken sample (it then waits for a keyframe).
    pub fn feed(&mut self, data: &[u8], format: &str) -> Result<Option<VideoImage>> {
        let codec = Codec::from_format(format)?;
        let keyframe = is_keyframe(codec, data);
        if !keyframe && !self.synced {
            return Ok(None);
        }
        if self.decoder.as_ref().is_none_or(|(current, _)| *current != codec) {
            // a new decoder knows no parameter sets: say so plainly when the keyframe lacks them
            (self.decoder, self.synced) = (None, false);
            check_decodable_alone(codec, data)?;
            self.decoder = Some((codec, match codec {
                Codec::H264 => Decoder::H264(h264::Stream::new()?),
                Codec::H265 => Decoder::H265(h265::Stream::new()?),
            }));
        }
        let Some((_, decoder)) = &mut self.decoder else { unreachable!("made above") };
        let picture = match decoder {
            Decoder::H264(stream) => stream.feed(data),
            Decoder::H265(stream) => stream.feed(data),
        };
        self.synced = picture.is_ok();
        let picture = picture.with_context(|| format!("{codec:?}: shows nothing until the next keyframe"))?;
        Ok(picture)
    }
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
        assert_like_rolled_pattern(image, 0, what);
    }

    /// Fails unless `image` is the pattern rolled `shift` columns right, to within a codec's loss.
    fn assert_like_rolled_pattern(image: &VideoImage, shift: usize, what: &str) {
        let rgb = crate::utils::png::to_rgb(&fixture("compressed/pattern.png")).unwrap();
        let row = rgb.width as usize * 3;
        let rolled: Vec<u8> = rgb.pixels.chunks(row).flat_map(|line| { let mut line = line.to_vec(); line.rotate_right(shift * 3); line }).collect();
        let expected = VideoImage::rgb8(rgb.width, rgb.height, rolled).unwrap().to_i420(320, 240).unwrap();
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

    /// `compressed/scroll.<format>` (gen_video_stream.py: 8 pictures rolled 16 more columns each, keyframes at 0 and 4)
    /// as access units, split at their delimiters; `None` where H.265 has no libde265.
    fn scroll(format: &str) -> Option<Vec<Vec<u8>>> {
        let codec = Codec::from_format(format).unwrap();
        if codec == Codec::H265 && crate::utils::h265::tests::decode_pattern().is_none() {
            return None;
        }
        let data = fixture(&format!("compressed/scroll.{format}"));
        let delimiter = match codec { Codec::H264 => 9, Codec::H265 => 35 };
        let mut starts: Vec<usize> = (0..data.len().saturating_sub(4)).filter(|&at| data[at..at + 3] == [0, 0, 1] && codec.nal_type(data[at + 3]) == delimiter).collect();
        starts.push(data.len());
        let units: Vec<Vec<u8>> = starts.windows(2).map(|pair| data[pair[0]..pair[1]].to_vec()).collect();
        assert_eq!(units.len(), 8, "{format}");
        Some(units)
    }

    #[test]
    fn a_stream_plays_its_predicted_frames_from_its_first_keyframe() {
        for format in ["h264", "h265"] {
            let Some(units) = scroll(format) else { continue };
            let mut pictures = Pictures::default();
            // joined at the second picture: predicted frames before a keyframe show nothing (and aren't errors)
            for unit in &units[1..4] {
                assert!(pictures.feed(unit, format).unwrap().is_none(), "{format}: nothing before a keyframe");
            }
            for (index, unit) in units.iter().enumerate().skip(4) {
                let picture = pictures.feed(unit, format).unwrap().unwrap_or_else(|| panic!("{format}: picture {index}"));
                assert_like_rolled_pattern(&picture, 16 * index, &format!("{format} picture {index}"));
            }
            let mut pictures = Pictures::default();
            for (index, unit) in units.iter().enumerate() {
                assert_like_rolled_pattern(&pictures.feed(unit, format).unwrap().unwrap(), 16 * index, &format!("{format} from the start, picture {index}"));
            }
        }
    }

    #[test]
    fn a_corrupt_sample_shows_nothing_until_the_next_keyframe() {
        for format in ["h264", "h265"] {
            let Some(units) = scroll(format) else { continue };
            let mut pictures = Pictures::default();
            pictures.feed(&units[0], format).unwrap().unwrap();
            pictures.feed(&units[1], format).unwrap().unwrap();
            let mut corrupt = units[2].clone();
            corrupt.truncate(corrupt.len() / 2);
            let error = pictures.feed(&corrupt, format).unwrap_err();
            assert!(format!("{error:#}").contains("until the next keyframe"), "{format}: {error:#}");
            assert!(pictures.feed(&units[3], format).unwrap().is_none(), "{format}: nothing after the corrupt sample");
            assert_like_rolled_pattern(&pictures.feed(&units[4], format).unwrap().unwrap(), 64, &format!("{format} keyframe after it"));
            assert_like_rolled_pattern(&pictures.feed(&units[5], format).unwrap().unwrap(), 80, &format!("{format} after it"));
        }
    }

    #[test]
    fn keyframe_only_streams_still_play() {
        let mut pictures = Pictures::default();
        for _ in 0..3 {
            assert_like_pattern(&pictures.feed(&fixture("compressed/pattern.h264"), "h264").unwrap().unwrap(), "h264 keyframes");
        }
        if crate::utils::h265::tests::decode_pattern().is_some() {
            for _ in 0..3 {
                assert_like_pattern(&pictures.feed(&fixture("compressed/pattern.h265"), "h265").unwrap().unwrap(), "h265 keyframes");
            }
        }
        let bare = Pictures::default().feed(&fixture("compressed/pattern_bare.h265"), "h265").unwrap_err().to_string();
        assert!(bare.contains("no VPS/SPS/PPS"), "{bare}");
    }

    #[test]
    fn planes_with_strides_crop_to_even_sizes() {
        // 3x3 with stride 4 (chroma 2x2 with stride 2): crops to 2x2
        let luma = [1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 0];
        let image = i420_from_planes(3, 3, [(&luma, 4), (&[10, 11, 12, 13], 2), (&[20, 21, 22, 23], 2)]).unwrap();
        assert_eq!((image.width(), image.height(), image.data()), (2, 2, &[1, 2, 4, 5, 10, 20][..]));
    }
}
