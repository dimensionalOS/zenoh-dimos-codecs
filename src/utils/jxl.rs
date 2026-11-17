//! JPEG XL decoding (jxl-oxide).

use crate::utils::raw_pixels::{Rgb8, samples_to_rgb};
use anyhow::{Result, ensure};

fn read(data: &[u8]) -> Result<jxl_oxide::JxlImage> {
    jxl_oxide::JxlImage::builder().read(data).map_err(|e| anyhow::anyhow!("jxl: {e}"))
}

fn render(image: &jxl_oxide::JxlImage) -> Result<jxl_oxide::Render> {
    image.render_frame(0).map_err(|e| anyhow::anyhow!("jxl: {e}"))
}

/// A JPEG XL image as RGB8.
pub fn to_rgb(data: &[u8]) -> Result<Rgb8> {
    let render = render(&read(data)?)?;
    let mut stream = render.stream_no_alpha();
    let (width, height, channels) = (stream.width(), stream.height(), stream.channels() as usize);
    let mut samples = vec![0u8; width as usize * height as usize * channels];
    stream.write_to_buffer(&mut samples);
    samples_to_rgb(&samples, channels, width, height)
}

/// A 9..16-bit single-channel JPEG XL as its u16 values (width, height, values).
pub fn to_u16(data: &[u8]) -> Result<(u32, u32, Vec<u16>)> {
    let image = read(data)?;
    let bits = image.image_header().metadata.bit_depth.bits_per_sample();
    ensure!(bits > 8 && bits <= 16, "jxl depth must be 9..16 bits per sample, got {bits}");
    let render = render(&image)?;
    let mut stream = render.stream_no_alpha();
    ensure!(stream.channels() == 1, "jxl depth must have one channel, got {}", stream.channels());
    let (width, height) = (stream.width(), stream.height());
    let mut values = vec![0u16; width as usize * height as usize];
    stream.write_to_buffer(&mut values);
    Ok((width, height, values))
}
