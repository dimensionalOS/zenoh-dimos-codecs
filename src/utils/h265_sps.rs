//! The picture size in an H.265 sequence parameter set.
//!
//! A passed-through access unit ([`bitstream::Bitstream::unit`](super::bitstream::Bitstream::unit)) is never
//! decoded, so nothing else knows how big its picture is: `foxglove_msgs/CompressedVideo` carries no size, and the
//! gateway reads the size off each encoded frame (it sets the stream's source size, the allocator's grant and the
//! per-frame metadata a page reads). So read it out of the SPS, which every keyframe carries in front of it.
//!
//! Only as much of the SPS as the size needs is parsed: everything up to `pic_height_in_luma_samples` and the
//! conformance window that crops it.

use anyhow::{Result, bail, ensure};

/// Bits of an SPS payload, with H.265's emulation-prevention bytes already gone.
pub(crate) struct Bits {
    /// the raw byte sequence payload (RBSP)
    bytes: Vec<u8>,
    /// how many bits have been read
    at: usize,
}

impl Bits {
    /// The RBSP of one NAL unit payload: every `0x03` that follows `0x00 0x00` is an emulation-prevention byte the
    /// encoder added so the bytes could not look like a start code, and is not part of the syntax.
    pub(crate) fn new(payload: &[u8]) -> Self {
        let mut bytes = Vec::with_capacity(payload.len());
        let mut zeros = 0;
        for &byte in payload {
            if zeros >= 2 && byte == 0x03 {
                zeros = 0;
                continue;
            }
            zeros = if byte == 0 { zeros + 1 } else { 0 };
            bytes.push(byte);
        }
        Bits { bytes, at: 0 }
    }

    /// `count` bits, most significant first; fails past the end rather than reading zeros, so a truncated SPS says so
    /// instead of giving a plausible size.
    pub(crate) fn bits(&mut self, count: usize) -> Result<u64> {
        ensure!(count <= 64, "{count} bits at once");
        ensure!(self.at + count <= self.bytes.len() * 8, "the sequence parameter set ends after {} bits, needed {}", self.bytes.len() * 8, self.at + count);
        let mut value = 0u64;
        for _ in 0..count {
            let byte = self.bytes[self.at / 8];
            value = (value << 1) | ((byte >> (7 - self.at % 8)) & 1) as u64;
            self.at += 1;
        }
        Ok(value)
    }

    /// One bit as a flag.
    pub(crate) fn flag(&mut self) -> Result<bool> {
        Ok(self.bits(1)? == 1)
    }

    /// An unsigned exp-Golomb code (`ue(v)`): the leading zeros say how many bits follow.
    pub(crate) fn ue(&mut self) -> Result<u64> {
        let mut leading = 0;
        while !self.flag()? {
            leading += 1;
            // 32 leading zeros is already past any size a picture can have: a desynced reader, not a huge number
            ensure!(leading <= 32, "the sequence parameter set is not exp-Golomb here ({leading} leading zeros)");
        }
        Ok((1u64 << leading) - 1 + self.bits(leading)?)
    }

    /// A signed exp-Golomb code (`se(v)`): maps the unsigned code k to 0, +1, -1, +2, -2, …
    pub(crate) fn se(&mut self) -> Result<i64> {
        let k = self.ue()?;
        Ok(if k % 2 == 1 { ((k + 1) / 2) as i64 } else { -((k / 2) as i64) })
    }

    /// `profile_tier_level`, which sits between the sub-layer count and the sizes and is skipped whole.
    fn skip_profile_tier_level(&mut self, max_sub_layers_minus1: u64) -> Result<()> {
        // general_profile_space(2) .. general_reserved_zero_43bits(43), general_inbld_flag(1), general_level_idc(8)
        self.bits(32)?;
        self.bits(32)?;
        self.bits(32)?;
        let mut present = Vec::with_capacity(max_sub_layers_minus1 as usize);
        for _ in 0..max_sub_layers_minus1 {
            present.push((self.flag()?, self.flag()?));
        }
        if max_sub_layers_minus1 > 0 {
            for _ in max_sub_layers_minus1..8 {
                self.bits(2)?;
            }
        }
        for (profile, level) in present {
            if profile {
                self.bits(32)?;
                self.bits(32)?;
                self.bits(24)?;
            }
            if level {
                self.bits(8)?;
            }
        }
        Ok(())
    }
}

/// The cropped picture size (width, height) in an H.265 SPS NAL unit, `payload` being the unit without its Annex-B start
/// code (so its two header bytes come first).
pub fn picture_size(payload: &[u8]) -> Result<(u32, u32)> {
    ensure!(payload.len() > 2, "an H.265 SPS is {} bytes", payload.len());
    let kind = (payload[0] >> 1) & 0x3f;
    ensure!(kind == 33, "NAL unit type {kind} is not an H.265 SPS (33)");
    let mut bits = Bits::new(&payload[2..]);
    // sps_video_parameter_set_id(4), sps_max_sub_layers_minus1(3), sps_temporal_id_nesting_flag(1)
    bits.bits(4)?;
    let max_sub_layers_minus1 = bits.bits(3)?;
    bits.flag()?;
    bits.skip_profile_tier_level(max_sub_layers_minus1)?;
    bits.ue()?; // sps_seq_parameter_set_id
    let chroma_format_idc = bits.ue()?;
    if chroma_format_idc == 3 {
        bits.flag()?; // separate_colour_plane_flag
    }
    let width = bits.ue()?;
    let height = bits.ue()?;
    // what one chroma sample covers, which is what a conformance window offset counts in
    let (sub_width, sub_height) = match chroma_format_idc {
        // monochrome and 4:4:4 are sampled like luma; 4:2:2 is halved across, 4:2:0 both ways
        0 | 3 => (1, 1),
        1 => (2, 2),
        2 => (2, 1),
        other => bail!("chroma_format_idc {other} is not 0-3"),
    };
    let (mut left, mut right, mut top, mut bottom) = (0, 0, 0, 0);
    if bits.flag()? {
        (left, right, top, bottom) = (bits.ue()?, bits.ue()?, bits.ue()?, bits.ue()?);
    }
    let cropped_width = width.checked_sub(sub_width * (left + right));
    let cropped_height = height.checked_sub(sub_height * (top + bottom));
    let (Some(cropped_width), Some(cropped_height)) = (cropped_width, cropped_height) else {
        bail!("an H.265 conformance window crops {width}x{height} away ({left}/{right}/{top}/{bottom})");
    };
    ensure!(cropped_width > 0 && cropped_height > 0 && cropped_width <= 16_384 && cropped_height <= 16_384, "an H.265 SPS says the picture is {cropped_width}x{cropped_height}");
    Ok((cropped_width as u32, cropped_height as u32))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::bitstream::{Annex, nal_units};
    use crate::utils::tests::fixture;

    /// The SPS of an Annex-B access unit.
    fn sps(data: &[u8]) -> Vec<u8> {
        nal_units(Annex::H265, data).into_iter().find(|(kind, _)| *kind == 33).expect("an SPS").1.to_vec()
    }

    #[test]
    fn reads_the_picture_size_out_of_a_real_sps() {
        // the fixture decodes (through libde265) to 320x240, so the SPS must say the same
        assert_eq!(picture_size(&sps(&fixture("compressed/pattern.h265"))).unwrap(), (320, 240));
    }

    /// The bytes an M20 publishes for a keyframe: ffmpeg's extradata (VPS/SPS/PPS) then the IDR.
    ///
    /// The size is **1280x720**, not the 800x600 the dimos camera asks its device for: the camera streams 720p
    /// whatever it is asked, and the bitstream is what the browser will decode. `agrees_with_libde265_on_real_hardware`
    /// pins that against the decoder itself, so this is not the parser's opinion.
    #[test]
    fn reads_the_m20_camera_s_own_sequence_parameter_set() {
        let au = fixture("compressed/m20_front_keyframe.h265");
        assert_eq!(nal_units(Annex::H265, &au).iter().map(|(kind, _)| *kind).collect::<Vec<_>>(), vec![32, 33, 34, 19]);
        assert_eq!(picture_size(&sps(&au)).unwrap(), (1280, 720));
    }

    /// Ground truth: libde265 decodes the real keyframe to the size this parser reads off its SPS. Without this, a
    /// parser bug and a wrong expectation could agree with each other.
    #[test]
    fn agrees_with_libde265_on_real_hardware() {
        let au = fixture("compressed/m20_front_keyframe.h265");
        let Ok(image) = crate::utils::h265::decode(&au) else {
            eprintln!("skipping: libde265 is not installed");
            return;
        };
        assert_eq!(picture_size(&sps(&au)).unwrap(), (image.width(), image.height()));
    }

    /// Two seconds off the camera's own RTSP stream: an IDR about every 30 frames, predicted frames between, start
    /// codes mixed, and — unlike what the dimos relay publishes — **no parameter sets in band at all**. That is why
    /// [`H265Passthrough`](crate::utils::video_passthrough::H265Passthrough) remembers them: a viewer that joins
    /// between the relay's extradata and the next keyframe has nothing else to start from.
    #[test]
    fn a_live_stream_is_mostly_predicted_frames_and_carries_no_parameter_sets() {
        let stream = fixture("compressed/m20_front_2s.h265");
        let units = nal_units(Annex::H265, &stream);
        let kinds: std::collections::BTreeMap<u8, usize> = units.iter().fold(Default::default(), |mut counts, (kind, _)| {
            *counts.entry(*kind).or_default() += 1;
            counts
        });
        assert_eq!(kinds.get(&33), None, "no sequence parameter set in band: {kinds:?}");
        assert!(kinds.get(&19).is_some_and(|idrs| *idrs >= 2), "at least two keyframes: {kinds:?}");
        // the predicted frames are the 29-in-30 the keyframes-only decoder drops today
        assert!(kinds.get(&1).is_some_and(|predicted| *predicted > 20), "mostly predicted frames: {kinds:?}");
    }

    #[test]
    fn says_why_it_cannot_read_one() {
        let short = picture_size(&[0x42]).unwrap_err().to_string();
        assert!(short.contains("is 1 bytes"), "{short}");
        // NAL type 1 (TRAIL_R), not an SPS
        let wrong = picture_size(&[0x02, 0x01, 0xaa, 0xbb]).unwrap_err().to_string();
        assert!(wrong.contains("is not an H.265 SPS"), "{wrong}");
        // an SPS header with nothing after it runs out of bits rather than inventing a size
        let truncated = picture_size(&[0x42, 0x01, 0x01]).unwrap_err().to_string();
        assert!(truncated.contains("ends after"), "{truncated}");
    }

    #[test]
    fn strips_emulation_prevention_bytes() {
        // 0x03 after two zero bytes is the encoder's, and must not reach the syntax
        let bits = Bits::new(&[0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x03, 0x00]);
        assert_eq!(bits.bytes, vec![0x00, 0x00, 0x01, 0x00, 0x00, 0x00]);
        // ... while 0x03 anywhere else is data
        let kept = Bits::new(&[0x03, 0x00, 0x03, 0x00]);
        assert_eq!(kept.bytes, vec![0x03, 0x00, 0x03, 0x00]);
    }

    #[test]
    fn reads_exp_golomb() {
        // 1 -> 0, 010 -> 1, 011 -> 2, 00100 -> 3
        let mut bits = Bits::new(&[0b1_010_011_0, 0b0100_0000]);
        assert_eq!([bits.ue().unwrap(), bits.ue().unwrap(), bits.ue().unwrap(), bits.ue().unwrap()], [0, 1, 2, 3]);
    }
}
