//! AV1, whose coded pictures are OBUs rather than Annex-B NAL units: a temporal unit is a run of OBUs that each carry
//! their own size, so reading one is exact rather than a search for start codes. The reading itself lives in
//! [`utils::av1`](crate::utils::av1); this is what makes it one of the [`Bitstream`]s.

use super::{Bitstream, Unit};
use crate::utils::av1;
use anyhow::Result;
use zenoh_gateway::VideoFormat;

/// AV1, temporal units in the low-overhead bitstream format.
pub struct AV1;

impl Bitstream for AV1 {
    fn format(&self) -> VideoFormat {
        VideoFormat::Av1
    }

    /// `"av01"` is what the ISO-BMFF/WebCodecs world calls it and it does turn up in publishers' `format` fields, so it
    /// is accepted beside the plain name rather than left to fail as an unknown codec.
    fn names(&self) -> &'static [&'static str] {
        &["av1", "av01"]
    }

    fn headers_called(&self) -> &'static str {
        "sequence header OBU"
    }

    fn unit(&self, data: &[u8]) -> Result<Unit> {
        let read = av1::temporal_unit(data)?;
        Ok(Unit { format: VideoFormat::Av1, data: read.data, keyframe: read.keyframe, headers: read.sequence_header, size: read.size })
    }

    fn unit_types(&self, data: &[u8]) -> Vec<u8> {
        av1::obus(data).map(|obus| obus.iter().map(|obu| obu.kind).collect()).unwrap_or_default()
    }

    fn types_called(&self) -> &'static str {
        "OBU types"
    }

    fn decode_alone(&self, data: &[u8]) -> Result<zenoh_gateway::VideoImage> {
        av1::decode(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn reads_a_temporal_unit_as_a_unit_like_any_other() {
        let whole = fixture("compressed/pattern.av1");
        let unit = AV1.unit(&whole).unwrap();
        assert_eq!((unit.format, unit.keyframe, unit.size), (VideoFormat::Av1, true, Some((320, 240))));
        assert!(unit.headers.is_some(), "the sequence header OBU is what a decoder starts from");
        // the temporal delimiter is dropped, and it is the only thing dropped: RTP's AV1 payload format does not carry one
        assert_eq!(unit.data.len(), whole.len() - av1::obus(&whole).unwrap()[0].bytes.len());
        assert_eq!(AV1.unit_types(&whole), vec![av1::OBU_TEMPORAL_DELIMITER, av1::OBU_SEQUENCE_HEADER, av1::OBU_FRAME]);
    }

    /// `unit_types` is only ever called to explain another failure, so a buffer too broken to read must come back as an
    /// empty list rather than panicking or failing inside an error message.
    #[test]
    fn unreadable_bytes_have_no_types_rather_than_failing() {
        // Annex-B H.264 read as OBUs: the first byte has no size field
        assert!(AV1.unit_types(&[0, 0, 0, 1, 0x67, 0x64]).is_empty());
        assert!(AV1.unit_types(&[]).is_empty());
        let error = AV1.unit(&[0, 0, 0, 1, 0x67, 0x64]).unwrap_err().to_string();
        assert!(error.contains("no size field"), "{error}");
    }
}
