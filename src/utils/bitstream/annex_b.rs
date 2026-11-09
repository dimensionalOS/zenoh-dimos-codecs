//! The Annex-B bitstreams: H.264 and H.265, which differ only in where a NAL unit keeps its type and which units are
//! keyframes and parameter sets. Everything else about reading one is shared, so it is written once here and the two
//! [`Bitstream`] impls are the table of what differs.

use super::{Bitstream, Unit};
use crate::utils::{h264_sps, h265_sps};
use anyhow::{Result, ensure};
use zenoh_gateway::VideoFormat;

/// Which Annex-B bitstream, for the parts of reading one that differ.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Annex {
    H264,
    H265,
}

/// Annex-B start code, four bytes, which [`unit`] puts in front of every NAL unit.
const START_CODE: [u8; 4] = [0, 0, 0, 1];
/// H.265 NAL unit types 48 and up are the RTP payload structures (RFC 7798 aggregation, fragmentation, PACI), so a NAL
/// unit of one cannot be carried inside another: a bitstream has no business holding one, and the packetizer refuses it.
const H265_RTP_ONLY_TYPES: u8 = 48;

impl Annex {
    /// A NAL unit's type, from its first byte.
    fn nal_type(self, first: u8) -> u8 {
        match self {
            Annex::H264 => first & 0x1f,
            Annex::H265 => (first >> 1) & 0x3f,
        }
    }

    /// (keyframe slice types, the parameter sets a keyframe needs, their names)
    fn keyframe_needs(self) -> (std::ops::RangeInclusive<u8>, &'static [(u8, &'static str)]) {
        match self {
            // IDR
            Annex::H264 => (5..=5, &[(7, "SPS"), (8, "PPS")]),
            // IRAP: BLA, IDR, CRA
            Annex::H265 => (16..=21, &[(32, "VPS"), (33, "SPS"), (34, "PPS")]),
        }
    }

    /// The sequence parameter set's NAL type: the only unit that says how big the picture is.
    fn sps_type(self) -> u8 {
        match self {
            Annex::H264 => 7,
            Annex::H265 => 33,
        }
    }

    /// The cropped picture size the sequence parameter set in `unit` gives.
    fn picture_size(self, unit: &[u8]) -> Result<(u32, u32)> {
        match self {
            Annex::H264 => h264_sps::picture_size(unit),
            Annex::H265 => h265_sps::picture_size(unit),
        }
    }

    /// A unit a decoder or packetizer has no use for, which [`unit`] drops.
    fn rtp_only(self, kind: u8) -> bool {
        self == Annex::H265 && kind >= H265_RTP_ONLY_TYPES
    }
}

/// The NAL units of an Annex-B access unit, in order: each one's type and its bytes without the start code.
///
/// Start codes are three or four bytes and one buffer mixes them (ffmpeg writes four ahead of parameter sets and three
/// ahead of slices), so each is found on its own: a `00 00 01` preceded by a `00` is the tail of a four-byte code, and
/// either way the unit begins just after the `01`. A unit's trailing zero bytes belong to no unit (Annex B's
/// `trailing_zero_8bits`, and a slice's `cabac_zero_word` padding) and are dropped, so the bytes handed on are the same
/// whichever width the next start code has.
pub fn nal_units(annex: Annex, data: &[u8]) -> Vec<(u8, &[u8])> {
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
            units.push((annex.nal_type(first), unit));
        }
    }
    units
}

/// Reads one Annex-B access unit.
///
/// Unlike the per-sample path this accepts predicted frames: the point of forwarding a stream is that the receiver's own
/// decoder keeps the reference chain, which is what makes every frame playable instead of one keyframe a second.
///
/// Every NAL unit is re-emitted behind a **four-byte** start code, whatever width it came with. That is not cosmetic:
/// the RTP packetizer this feeds (`rtc-rtp`'s `HevcPayloader`) looks for four-byte start codes first and, if it finds
/// any, reads *every* unit as if it had one — so an access unit that mixes widths has units merged and NAL headers read
/// a byte late. Publishers do mix them (ffmpeg writes four ahead of parameter sets and three ahead of slices, which is
/// exactly what the M20's camera relay sends), and the split above already handles both, so normalising costs one copy
/// and removes the whole class of bug without patching the packetizer.
fn unit(annex: Annex, format: VideoFormat, data: &[u8]) -> Result<Unit> {
    let units = nal_units(annex, data);
    ensure!(!units.is_empty(), "{format:?} data has no Annex-B start code (00 00 01)");
    let (keyframes, wanted) = annex.keyframe_needs();
    let keyframe = units.iter().any(|(kind, _)| keyframes.contains(kind));
    let mut headers = Vec::new();
    let mut normalized = Vec::with_capacity(data.len() + 4);
    for (kind, unit) in &units {
        if annex.rtp_only(*kind) {
            continue;
        }
        if wanted.iter().any(|(set, _)| set == kind) {
            headers.extend_from_slice(&START_CODE);
            headers.extend_from_slice(unit);
        }
        normalized.extend_from_slice(&START_CODE);
        normalized.extend_from_slice(unit);
    }
    ensure!(!normalized.is_empty(), "{format:?} data has no NAL unit a decoder could be sent (types {:?})", units.iter().map(|(kind, _)| *kind).collect::<Vec<_>>());
    // the size lives in the sequence parameter set, the only place a forwarded stream carries it
    let size = units.iter().find(|(kind, _)| *kind == annex.sps_type()).map(|(_, unit)| annex.picture_size(unit)).transpose()?;
    Ok(Unit { format, data: normalized, keyframe, headers: (!headers.is_empty()).then_some(headers), size })
}

/// H.264 (AVC), Annex-B access units.
pub struct H264;

/// H.265 (HEVC), Annex-B access units.
pub struct H265;

impl Bitstream for H264 {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn names(&self) -> &'static [&'static str] {
        &["h264", "avc"]
    }

    fn headers_called(&self) -> &'static str {
        "SPS/PPS"
    }

    fn unit(&self, data: &[u8]) -> Result<Unit> {
        unit(Annex::H264, VideoFormat::H264, data)
    }

    fn unit_types(&self, data: &[u8]) -> Vec<u8> {
        nal_units(Annex::H264, data).into_iter().map(|(kind, _)| kind).collect()
    }

    fn types_called(&self) -> &'static str {
        "NAL types"
    }

    fn decode_alone(&self, data: &[u8]) -> Result<zenoh_gateway::VideoImage> {
        crate::utils::h264::decode(data)
    }
}

impl Bitstream for H265 {
    fn format(&self) -> VideoFormat {
        VideoFormat::H265
    }

    fn names(&self) -> &'static [&'static str] {
        &["h265", "hevc"]
    }

    fn headers_called(&self) -> &'static str {
        "VPS/SPS/PPS"
    }

    fn unit(&self, data: &[u8]) -> Result<Unit> {
        unit(Annex::H265, VideoFormat::H265, data)
    }

    fn unit_types(&self, data: &[u8]) -> Vec<u8> {
        nal_units(Annex::H265, data).into_iter().map(|(kind, _)| kind).collect()
    }

    fn types_called(&self) -> &'static str {
        "NAL types"
    }

    fn decode_alone(&self, data: &[u8]) -> Result<zenoh_gateway::VideoImage> {
        crate::utils::h265::decode(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    /// The same NAL units behind alternating three- and four-byte start codes, as ffmpeg's output mixes them.
    fn with_mixed_start_codes(annex: Annex, data: &[u8]) -> Vec<u8> {
        let mut mixed = Vec::with_capacity(data.len() + 8);
        for (index, (_, unit)) in nal_units(annex, data).into_iter().enumerate() {
            mixed.extend_from_slice(if index % 2 == 0 { &[0, 0, 1][..] } else { &[0, 0, 0, 1][..] });
            mixed.extend_from_slice(unit);
        }
        mixed
    }

    /// The packetizer this feeds reads every start code at the width of the first four-byte one it finds, so an access
    /// unit must not reach it with widths mixed, however it arrived.
    #[test]
    fn start_codes_come_out_four_bytes_wide_whatever_went_in() {
        let data = fixture("compressed/pattern.h265");
        let units = nal_units(Annex::H265, &data).len();
        assert!(units >= 4, "the fixture has {units} NAL units to mix");
        let mixed = with_mixed_start_codes(Annex::H265, &data);
        assert_ne!(mixed, data, "the mixed version really differs");
        let from_mixed = H265.unit(&mixed).unwrap();
        assert_eq!(from_mixed, H265.unit(&data).unwrap(), "both normalise to the same access unit");
        assert_eq!(from_mixed.data.windows(4).filter(|window| *window == START_CODE).count(), units, "one four-byte start code per NAL unit");
        assert_eq!(from_mixed.size, Some((320, 240)), "and the SPS still reads");
    }

    #[test]
    fn an_access_unit_keeps_predicted_frames_and_reads_what_it_can() {
        // a bare TRAIL_R slice: what the per-sample path refuses and forwarding exists to carry
        let predicted = H265.unit(&[0, 0, 1, 0x02, 0x01, 0xaa]).unwrap();
        assert_eq!(predicted, Unit { format: VideoFormat::H265, data: vec![0, 0, 0, 1, 0x02, 0x01, 0xaa], keyframe: false, headers: None, size: None });
        let keyframe = H265.unit(&fixture("compressed/pattern.h265")).unwrap();
        assert!(keyframe.keyframe && keyframe.size == Some((320, 240)));
        let headers = keyframe.headers.expect("the fixture carries its VPS/SPS/PPS");
        assert_eq!(H265.unit_types(&headers), vec![32, 33, 34]);
        // and a sequence parameter set it cannot read is said so, not passed off as a stream with no size
        let unreadable = H265.unit(&[0, 0, 1, 0x42, 0x01, 0x01]).unwrap_err().to_string();
        assert!(unreadable.contains("sequence parameter set ends after"), "{unreadable}");
    }

    #[test]
    fn h264_and_h265_read_their_own_types_and_sizes() {
        let h264 = H264.unit(&fixture("compressed/pattern.h264")).unwrap();
        assert_eq!((h264.format, h264.size, h264.keyframe), (VideoFormat::H264, Some((320, 240)), true));
        assert_eq!(H264.unit_types(&fixture("compressed/pattern.h264")), vec![7, 8, 6, 5], "SPS, PPS, SEI, IDR");
        // an H.265 trailing picture (TRAIL_R, type 1) is predicted, and H.264's type 1 is too, but they are read from
        // different bits of the first byte: the same buffer must not read the same in both
        assert_ne!(H264.unit_types(&fixture("compressed/pattern.h265")), H265.unit_types(&fixture("compressed/pattern.h265")));
    }

    /// An H.265 NAL unit of an RTP payload type has no place in a bitstream, and the packetizer refuses one, so it is
    /// dropped here rather than forwarded into a failure further on.
    #[test]
    fn an_rtp_only_nal_unit_is_dropped_before_the_packetizer_sees_it() {
        // type 49 (fragmentation unit) next to a real slice: (49 << 1) = 0x62
        let unit = H265.unit(&[0, 0, 1, 0x62, 0x01, 0xff, 0, 0, 1, 0x02, 0x01, 0xaa]).unwrap();
        assert_eq!(unit.data, vec![0, 0, 0, 1, 0x02, 0x01, 0xaa], "only the slice is forwarded");
        // and a buffer with nothing but one says so instead of forwarding an empty access unit
        let nothing = H265.unit(&[0, 0, 1, 0x62, 0x01, 0xff]).unwrap_err().to_string();
        assert!(nothing.contains("no NAL unit a decoder could be sent"), "{nothing}");
    }
}
