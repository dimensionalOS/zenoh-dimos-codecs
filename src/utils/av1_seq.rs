//! The picture size in an AV1 sequence header OBU, for a passthrough stream: the gateway wants a size with every frame
//! and only a sequence header carries one, exactly as [`h265_sps`](super::h265_sps) and [`h264_sps`](super::h264_sps)
//! read it off an SPS.
//!
//! The size wanted is `max_frame_width_minus_1`/`max_frame_height_minus_1`, which sit after the operating points, so
//! the fields in front of them have to be walked even though none of them is used. Unlike H.264/H.265 there is no
//! emulation prevention to undo: an OBU's payload is the bits as written (AV1 spec 5.5.1).

use anyhow::{Result, ensure};

/// Reads big-endian bits out of an OBU payload, MSB first, which is how AV1 writes `f(n)` (spec 4.10.2).
pub(crate) struct Bits<'a> {
    data: &'a [u8],
    /// bits already read
    at: usize,
}

impl<'a> Bits<'a> {
    pub(crate) fn new(data: &'a [u8]) -> Self {
        Bits { data, at: 0 }
    }

    /// `f(n)` for n up to 32.
    pub(crate) fn f(&mut self, count: usize) -> Result<u32> {
        debug_assert!(count <= 32, "f({count}) is wider than the u32 it returns");
        ensure!(self.at + count <= self.data.len() * 8, "AV1 sequence header ends mid-field (wanted {count} bits at bit {})", self.at);
        let mut value = 0u32;
        for _ in 0..count {
            let byte = self.data[self.at / 8];
            value = (value << 1) | ((byte >> (7 - self.at % 8)) & 1) as u32;
            self.at += 1;
        }
        Ok(value)
    }

    /// One bit as a flag.
    fn flag(&mut self) -> Result<bool> {
        Ok(self.f(1)? == 1)
    }

    /// `uvlc()`: leading zeros, then that many bits after the terminating one (spec 4.10.3).
    fn uvlc(&mut self) -> Result<u64> {
        let mut zeros = 0;
        while !self.flag()? {
            zeros += 1;
            ensure!(zeros < 32, "AV1 uvlc runs past 32 leading zeros");
        }
        if zeros == 0 {
            return Ok(0);
        }
        Ok((1u64 << zeros) - 1 + self.f(zeros)? as u64)
    }
}

/// The stream's picture size from a sequence header OBU's payload (the bytes after its header and size field).
///
/// This is `max_frame_width`/`max_frame_height`: the largest a frame in the sequence may be, and the size every frame
/// is unless it overrides it. It is the one size a passthrough stream states, so it is what the track is sized by.
pub fn picture_size(payload: &[u8]) -> Result<(u32, u32)> {
    let mut bits = Bits::new(payload);
    let _seq_profile = bits.f(3)?;
    let _still_picture = bits.flag()?;
    let reduced_still_picture_header = bits.flag()?;
    if reduced_still_picture_header {
        let _seq_level_idx = bits.f(5)?;
    } else {
        // timing_info_present_flag, and the decoder model that only a stream with timing info can have
        let mut buffer_delay_length = 0;
        let mut decoder_model = false;
        if bits.flag()? {
            // timing_info(): num_units_in_display_tick, time_scale, equal_picture_interval
            let (_units, _scale) = (bits.f(32)?, bits.f(32)?);
            if bits.flag()? {
                let _num_ticks_per_picture_minus_1 = bits.uvlc()?;
            }
            decoder_model = bits.flag()?;
            if decoder_model {
                buffer_delay_length = bits.f(5)? as usize + 1;
                let _num_units_in_decoding_tick = bits.f(32)?;
                let _buffer_removal_time_length_minus_1 = bits.f(5)?;
                let _frame_presentation_time_length_minus_1 = bits.f(5)?;
            }
        }
        let initial_display_delay = bits.flag()?;
        let operating_points = bits.f(5)? + 1;
        for _ in 0..operating_points {
            let _operating_point_idc = bits.f(12)?;
            let level = bits.f(5)?;
            if level > 7 {
                let _seq_tier = bits.flag()?;
            }
            if decoder_model && bits.flag()? {
                // operating_parameters_info(): decoder and encoder buffer delay, then low_delay_mode_flag
                let (_decoder_delay, _encoder_delay) = (bits.f(buffer_delay_length)?, bits.f(buffer_delay_length)?);
                let _low_delay_mode = bits.flag()?;
            }
            if initial_display_delay && bits.flag()? {
                let _initial_display_delay_minus_1 = bits.f(4)?;
            }
        }
    }
    let width_bits = bits.f(4)? as usize + 1;
    let height_bits = bits.f(4)? as usize + 1;
    let width = bits.f(width_bits)? + 1;
    let height = bits.f(height_bits)? + 1;
    ensure!(width > 0 && height > 0, "AV1 sequence header gives an empty picture ({width}x{height})");
    Ok((width, height))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::av1;
    use crate::utils::tests::fixture;

    /// The sequence header OBU's payload out of a temporal unit.
    fn sequence_header(data: &[u8]) -> Vec<u8> {
        av1::obus(data).unwrap().into_iter().find(|obu| obu.kind == av1::OBU_SEQUENCE_HEADER).expect("a sequence header").payload.to_vec()
    }

    #[test]
    fn reads_the_size_of_the_pattern() {
        let payload = sequence_header(&fixture("compressed/pattern.av1"));
        assert_eq!(picture_size(&payload).unwrap(), (320, 240));
    }

    #[test]
    fn a_truncated_sequence_header_says_so() {
        let payload = sequence_header(&fixture("compressed/pattern.av1"));
        let error = picture_size(&payload[..2]).unwrap_err().to_string();
        assert!(error.contains("ends mid-field"), "{error}");
    }

    #[test]
    fn uvlc_reads_the_spec_values() {
        // 1 -> 0; 010 -> 1; 011 -> 2; 00100 -> 3
        assert_eq!(Bits::new(&[0b1000_0000]).uvlc().unwrap(), 0);
        assert_eq!(Bits::new(&[0b0100_0000]).uvlc().unwrap(), 1);
        assert_eq!(Bits::new(&[0b0110_0000]).uvlc().unwrap(), 2);
        assert_eq!(Bits::new(&[0b0010_0000]).uvlc().unwrap(), 3);
    }
}
