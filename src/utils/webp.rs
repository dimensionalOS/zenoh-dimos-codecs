//! WebP decoding (image-webp).

use crate::utils::raw_pixels::{Rgb8, samples_to_rgb};
use anyhow::{Context, Result};
use std::io::Cursor;

/// A WebP as RGB8.
pub fn to_rgb(data: &[u8]) -> Result<Rgb8> {
    let mut decoder = image_webp::WebPDecoder::new(Cursor::new(data)).map_err(|e| anyhow::anyhow!("webp: {e}"))?;
    let (width, height) = decoder.dimensions();
    let channels = if decoder.has_alpha() { 4 } else { 3 };
    let mut samples = vec![0u8; decoder.output_buffer_size().context("webp: image too large")?];
    decoder.read_image(&mut samples).map_err(|e| anyhow::anyhow!("webp: {e}"))?;
    samples_to_rgb(&samples, channels, width, height)
}
