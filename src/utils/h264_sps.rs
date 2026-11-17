//! The picture size in an H.264 sequence parameter set (NAL unit type 7).
//!
//! The H.264 twin of [`h265_sps`](super::h265_sps): a passed-through access unit
//! ([`bitstream::Bitstream::unit`](super::bitstream::Bitstream::unit)) is never decoded here, so the
//! gateway reads the picture size off the SPS that every keyframe carries in front of it. Only as much of the SPS as
//! the size needs is parsed (up to `frame_cropping`), reusing the RBSP bit reader from `h265_sps`.

use super::h265_sps::Bits;
use anyhow::{Result, ensure};

/// Profiles that carry the chroma/bit-depth/scaling-list block before the frame-num fields (High and friends).
fn has_chroma_block(profile_idc: u64) -> bool {
    matches!(profile_idc, 100 | 110 | 122 | 244 | 44 | 83 | 86 | 118 | 128 | 138 | 139 | 134 | 135)
}

/// Skips a scaling list of `size` entries (`scaling_list()` in the spec): runs the delta chain until it ends.
fn skip_scaling_list(bits: &mut Bits, size: usize) -> Result<()> {
    let mut last = 8i64;
    let mut next = 8i64;
    for _ in 0..size {
        if next != 0 {
            let delta = bits.se()?;
            next = (last + delta + 256) % 256;
        }
        last = if next == 0 { last } else { next };
    }
    Ok(())
}

/// The cropped picture size (width, height) in an H.264 SPS NAL unit, `payload` being the unit without its Annex-B start
/// code (so its one header byte comes first).
pub fn picture_size(payload: &[u8]) -> Result<(u32, u32)> {
    ensure!(!payload.is_empty(), "an H.264 SPS is empty");
    let kind = payload[0] & 0x1f;
    ensure!(kind == 7, "NAL unit type {kind} is not an H.264 SPS (7)");
    let mut bits = Bits::new(&payload[1..]);
    let profile_idc = bits.bits(8)?;
    bits.bits(8)?; // constraint_set flags + reserved_zero_2bits
    bits.bits(8)?; // level_idc
    bits.ue()?; // seq_parameter_set_id
    let mut chroma_format_idc = 1; // 4:2:0 unless a high profile says otherwise
    if has_chroma_block(profile_idc) {
        chroma_format_idc = bits.ue()?;
        if chroma_format_idc == 3 {
            bits.flag()?; // separate_colour_plane_flag
        }
        bits.ue()?; // bit_depth_luma_minus8
        bits.ue()?; // bit_depth_chroma_minus8
        bits.flag()?; // qpprime_y_zero_transform_bypass_flag
        if bits.flag()? {
            // seq_scaling_matrix_present_flag
            let lists = if chroma_format_idc != 3 { 8 } else { 12 };
            for index in 0..lists {
                if bits.flag()? {
                    // seq_scaling_list_present_flag[index]
                    skip_scaling_list(&mut bits, if index < 6 { 16 } else { 64 })?;
                }
            }
        }
    }
    bits.ue()?; // log2_max_frame_num_minus4
    let pic_order_cnt_type = bits.ue()?;
    if pic_order_cnt_type == 0 {
        bits.ue()?; // log2_max_pic_order_cnt_lsb_minus4
    } else if pic_order_cnt_type == 1 {
        bits.flag()?; // delta_pic_order_always_zero_flag
        bits.se()?; // offset_for_non_ref_pic
        bits.se()?; // offset_for_top_to_bottom_field
        let cycle = bits.ue()?;
        ensure!(cycle <= 255, "num_ref_frames_in_pic_order_cnt_cycle {cycle} is implausible");
        for _ in 0..cycle {
            bits.se()?; // offset_for_ref_frame[i]
        }
    }
    bits.ue()?; // max_num_ref_frames
    bits.flag()?; // gaps_in_frame_num_value_allowed_flag
    let width_mbs = bits.ue()? + 1;
    let height_map_units = bits.ue()? + 1;
    let frame_mbs_only = bits.flag()?;
    if !frame_mbs_only {
        bits.flag()?; // mb_adaptive_frame_field_flag
    }
    bits.flag()?; // direct_8x8_inference_flag
    let (mut left, mut right, mut top, mut bottom) = (0u64, 0u64, 0u64, 0u64);
    if bits.flag()? {
        // frame_cropping_flag
        (left, right, top, bottom) = (bits.ue()?, bits.ue()?, bits.ue()?, bits.ue()?);
    }
    let field = if frame_mbs_only { 1 } else { 2 };
    let width_px = width_mbs * 16;
    let height_px = field * height_map_units * 16;
    // crop offsets count in chroma samples: CropUnitX = SubWidthC, CropUnitY = SubHeightC * (2 - frame_mbs_only)
    let (sub_width, sub_height) = match chroma_format_idc {
        0 => (1, 1), // monochrome: ChromaArrayType 0, crop unit (1, 2 - frame_mbs_only)
        1 => (2, 2), // 4:2:0
        2 => (2, 1), // 4:2:2
        _ => (1, 1), // 4:4:4
    };
    let (crop_x, crop_y) = (sub_width, sub_height * field);
    let cropped_width = width_px.checked_sub(crop_x * (left + right));
    let cropped_height = height_px.checked_sub(crop_y * (top + bottom));
    let (Some(cropped_width), Some(cropped_height)) = (cropped_width, cropped_height) else {
        anyhow::bail!("an H.264 frame-cropping window crops {width_px}x{height_px} away ({left}/{right}/{top}/{bottom})");
    };
    ensure!(
        cropped_width > 0 && cropped_height > 0 && cropped_width <= 16_384 && cropped_height <= 16_384,
        "an H.264 SPS says the picture is {cropped_width}x{cropped_height}"
    );
    Ok((cropped_width as u32, cropped_height as u32))
}
