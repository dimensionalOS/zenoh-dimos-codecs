//! Compressed video (`foxglove_msgs/CompressedVideo` data): one Annex-B access unit of H.264 or H.265 to a picture for
//! the video encoder. Each sample is decoded on its own (the gateway decodes only the samples it sends), so a sample must
//! be a keyframe carrying its parameter sets: a stream with predicted frames, or parameter sets sent once, can't play.

use crate::utils::{h264, h265, h265_sps};
use anyhow::{Result, bail, ensure};
use zenoh_gateway::{Channel, VideoImage};

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

/// The NAL units of an Annex-B access unit, in order: each one's type and its bytes without the start code.
///
/// Start codes are three or four bytes and one buffer mixes them (ffmpeg writes four ahead of parameter sets and three
/// ahead of slices), so each is found on its own: a `00 00 01` preceded by a `00` is the tail of a four-byte code, and
/// either way the unit begins just after the `01`. A unit's trailing zero bytes belong to no unit (Annex B's
/// `trailing_zero_8bits`, and a slice's `cabac_zero_word` padding) and are dropped, so the bytes handed on are the same
/// whichever width the next start code has.
pub(crate) fn nal_units(codec: Codec, data: &[u8]) -> Vec<(u8, &[u8])> {
    let mut starts = Vec::new();
    let mut at = 0;
    while let Some(found) = data.get(at..).and_then(|rest| rest.windows(3).position(|window| window == [0, 0, 1])) {
        at += found + 3;
        starts.push(at);
    }
    let mut units = Vec::with_capacity(starts.len());
    for (index, &start) in starts.iter().enumerate() {
        // up to the next start code, less that code's leading zeros, which this unit does not own
        let end = starts.get(index + 1).map_or(data.len(), |&next| next - 3);
        let unit = &data[start..end];
        let unit = &unit[..unit.iter().rposition(|&byte| byte != 0).map_or(0, |last| last + 1)];
        if let Some(&first) = unit.first() {
            units.push((codec.nal_type(first), unit));
        }
    }
    units
}

/// The types of the NAL units in an Annex-B access unit, in order.
fn nal_types(codec: Codec, data: &[u8]) -> Vec<u8> {
    nal_units(codec, data).into_iter().map(|(kind, _)| kind).collect()
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

/// Whether `channel` wants the access unit itself ([`to_access_unit`]) rather than the picture in it ([`to_video`]):
/// a video channel of a format the gateway only passes through.
pub fn passes_through(channel: Channel) -> bool {
    matches!(channel, Channel::Video(format) if format.passthrough_only())
}

/// One access unit as it arrived, for a video channel the gateway passes through
/// ([`VideoFormat::passthrough_only`](zenoh_gateway::VideoFormat::passthrough_only)): nothing decodes it here, so it
/// carries what the gateway would otherwise have learnt by decoding it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AccessUnit {
    /// which codec's bitstream `data` is
    pub codec: Codec,
    /// the access unit's NAL units, each behind a four-byte Annex-B start code; see [`to_access_unit`] on why they are
    /// all four bytes even when the publisher mixed three and four
    pub data: Vec<u8>,
    /// an IRAP picture (H.265 BLA/IDR/CRA, H.264 IDR): a decoder can start here, given the parameter sets
    pub keyframe: bool,
    /// the parameter sets in front of it (H.265 VPS/SPS/PPS, H.264 SPS/PPS), Annex-B, if it carries them: a keyframe
    /// that arrives without them is only decodable after they are put back ([`parameter_sets`](Self::parameter_sets))
    pub parameter_sets: Option<Vec<u8>>,
    /// the cropped picture size from the sequence parameter set, when this unit carries one
    pub size: Option<(u32, u32)>,
}

/// Annex-B start code, four bytes, which [`to_access_unit`] puts in front of every NAL unit.
const START_CODE: [u8; 4] = [0, 0, 0, 1];
/// NAL unit types 48 and up are the RTP payload structures (RFC 7798 aggregation, fragmentation, PACI), so a NAL unit of
/// one cannot be carried in another: a bitstream has no business holding one, and the packetizer refuses it.
const H265_RTP_ONLY_TYPES: u8 = 48;

/// The access unit in `data` for a passthrough video channel, with what can be read off it without decoding.
///
/// Unlike [`to_video`] this accepts predicted frames: the point of passing a stream through is that the browser's own
/// decoder keeps the reference chain, which is what makes every frame playable instead of one keyframe a second.
///
/// Every NAL unit is re-emitted behind a **four-byte** start code, whatever width it came with. That is not cosmetic:
/// the RTP packetizer this feeds (`rtc-rtp`'s `HevcPayloader`) looks for four-byte start codes first and, if it finds
/// any, reads *every* unit as if it had one — so an access unit that mixes widths has units merged and NAL headers read
/// a byte late. Publishers do mix them (ffmpeg writes four ahead of parameter sets and three ahead of slices, which is
/// exactly what the M20's camera relay sends), and the split here already handles both, so normalising costs one copy
/// and removes the whole class of bug without patching the packetizer.
pub fn to_access_unit(data: &[u8], format: &str) -> Result<AccessUnit> {
    let codec = Codec::from_format(format)?;
    let units = nal_units(codec, data);
    ensure!(!units.is_empty(), "{codec:?} data has no Annex-B start code (00 00 01)");
    let (keyframes, wanted) = codec.keyframe_needs();
    let keyframe = units.iter().any(|(kind, _)| keyframes.contains(kind));
    let mut parameter_sets = Vec::new();
    let mut normalized = Vec::with_capacity(data.len() + 4);
    for (kind, unit) in &units {
        if codec == Codec::H265 && *kind >= H265_RTP_ONLY_TYPES {
            continue;
        }
        if wanted.iter().any(|(set, _)| set == kind) {
            parameter_sets.extend_from_slice(&START_CODE);
            parameter_sets.extend_from_slice(unit);
        }
        normalized.extend_from_slice(&START_CODE);
        normalized.extend_from_slice(unit);
    }
    ensure!(!normalized.is_empty(), "{codec:?} data has no NAL unit a decoder could be sent (types {:?})", units.iter().map(|(kind, _)| *kind).collect::<Vec<_>>());
    // the size lives in the SPS (H.265 type 33, H.264 type 7); only H.265 is passed through today, so only its is read
    let size = match codec {
        Codec::H265 => units.iter().find(|(kind, _)| *kind == 33).map(|(_, unit)| h265_sps::picture_size(unit)).transpose()?,
        Codec::H264 => None,
    };
    Ok(AccessUnit { codec, data: normalized, keyframe, parameter_sets: (!parameter_sets.is_empty()).then_some(parameter_sets), size })
}

/// The bytes a decoder can be handed for `unit`, and whether it can start from them.
///
/// The camera's own stream carries no parameter sets in band: the only ones anybody sees are the extradata the publisher
/// prepends to a keyframe. So `sets` remembers the newest, and a keyframe that arrives without them gets them put back
/// in front — otherwise a consumer that joined between one keyframe and the next has nothing to start from, and a
/// decoder would wait for a keyframe that never comes complete.
pub fn decodable(unit: &AccessUnit, sets: &mut Option<Vec<u8>>) -> (Vec<u8>, bool) {
    if let Some(carried) = &unit.parameter_sets {
        *sets = Some(carried.clone());
    }
    match (unit.keyframe, unit.parameter_sets.is_none(), sets.as_ref()) {
        // a keyframe without its parameter sets, and some remembered: put them in front and it decodes on its own
        (true, true, Some(remembered)) => ([remembered.as_slice(), unit.data.as_slice()].concat(), true),
        // a keyframe that carries its own, or one with none to be had (nothing can start from that), or any other frame
        (keyframe, _, sets) => (unit.data.clone(), keyframe && sets.is_some()),
    }
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

    /// The same NAL units behind alternating three- and four-byte start codes, as ffmpeg's output mixes them.
    fn with_mixed_start_codes(codec: Codec, data: &[u8]) -> Vec<u8> {
        let mut mixed = Vec::with_capacity(data.len() + 8);
        for (index, (_, unit)) in nal_units(codec, data).into_iter().enumerate() {
            mixed.extend_from_slice(if index % 2 == 0 { &[0, 0, 1][..] } else { &[0, 0, 0, 1][..] });
            mixed.extend_from_slice(unit);
        }
        mixed
    }

    /// The packetizer `to_access_unit` feeds reads every start code at the width of the first four-byte one it finds, so
    /// an access unit must not reach it with widths mixed, however it arrived.
    #[test]
    fn start_codes_come_out_four_bytes_wide_whatever_went_in() {
        let data = fixture("compressed/pattern.h265");
        let units = nal_units(Codec::H265, &data).len();
        assert!(units >= 4, "the fixture has {units} NAL units to mix");
        let mixed = with_mixed_start_codes(Codec::H265, &data);
        assert_ne!(mixed, data, "the mixed version really differs");
        let from_mixed = to_access_unit(&mixed, "h265").unwrap();
        assert_eq!(from_mixed, to_access_unit(&data, "h265").unwrap(), "both normalise to the same access unit");
        assert_eq!(from_mixed.data.windows(4).filter(|window| *window == START_CODE).count(), units, "one four-byte start code per NAL unit");
        assert_eq!(from_mixed.size, Some((320, 240)), "and the SPS still reads");
    }

    #[test]
    fn an_access_unit_keeps_predicted_frames_and_reads_what_it_can() {
        // a bare TRAIL_R slice: what `to_video` refuses and passthrough exists to carry
        let predicted = to_access_unit(&[0, 0, 1, 0x02, 0x01, 0xaa], "h265").unwrap();
        assert_eq!(predicted, AccessUnit { codec: Codec::H265, data: vec![0, 0, 0, 1, 0x02, 0x01, 0xaa], keyframe: false, parameter_sets: None, size: None });
        let keyframe = to_access_unit(&fixture("compressed/pattern.h265"), "h265").unwrap();
        assert!(keyframe.keyframe && keyframe.size == Some((320, 240)));
        let sets = keyframe.parameter_sets.expect("the fixture carries its VPS/SPS/PPS");
        assert_eq!(nal_types(Codec::H265, &sets), vec![32, 33, 34]);
        // and a sequence parameter set it cannot read is said so, not passed off as a stream with no size
        let unreadable = to_access_unit(&[0, 0, 1, 0x42, 0x01, 0x01], "h265").unwrap_err().to_string();
        assert!(unreadable.contains("sequence parameter set ends after"), "{unreadable}");
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
