//! JPEG decoding (libjpeg-turbo with the `turbojpeg` feature, else zune-jpeg): to RGB8, or a YCbCr JPEG straight to
//! I420 for the video path; and encoding (jpeg-encoder).

use crate::utils::raw_pixels::{Rgb8, samples_to_rgb};
use anyhow::{Context, Result, ensure};
use zenoh_gateway::VideoImage;
use zune_jpeg::zune_core::bytestream::ZCursor;
use zune_jpeg::zune_core::colorspace::ColorSpace;
use zune_jpeg::zune_core::options::DecoderOptions;

/// JPEG quality at the allocator's quality 0.
const LOWEST_JPEG_QUALITY: f64 = 20.0;
/// JPEG quality at quality 1: past ~95 "the compressed file size goes up dramatically for hardly any gain" (libjpeg's usage.txt).
const HIGHEST_JPEG_QUALITY: f64 = 95.0;

/// RGB8 as a JPEG at `quality` (0..1 → JPEG quality 20..95).
pub fn encode(image: &Rgb8, quality: f64) -> Result<Vec<u8>> {
    let (width, height) = (u16::try_from(image.width).context("jpeg: wider than 65535")?, u16::try_from(image.height).context("jpeg: taller than 65535")?);
    let mut out = Vec::new();
    let jpeg_quality = (LOWEST_JPEG_QUALITY + (HIGHEST_JPEG_QUALITY - LOWEST_JPEG_QUALITY) * quality.clamp(0.0, 1.0)).round() as u8;
    jpeg_encoder::Encoder::new(&mut out, jpeg_quality).encode(&image.pixels, width, height, jpeg_encoder::ColorType::Rgb).map_err(|e| anyhow::anyhow!("jpeg: {e}"))?;
    Ok(out)
}

/// A JPEG as RGB8.
pub fn to_rgb(data: &[u8]) -> Result<Rgb8> {
    #[cfg(feature = "turbojpeg")]
    return turbo::to_rgb(data);
    #[cfg(not(feature = "turbojpeg"))]
    zune_to_rgb(data)
}

/// JFIF's full-range YCbCr as BT.601 limited-range I420 (what the H.264 stream is tagged as, and
/// what zenoh-gateway's RGB conversion produces), chroma from each 2x2 block's mean.
/// `None` for a JPEG that isn't YCbCr or has an odd side.
pub fn to_i420(data: &[u8]) -> Result<Option<VideoImage>> {
    #[cfg(feature = "turbojpeg")]
    return turbo::to_i420(data);
    #[cfg(not(feature = "turbojpeg"))]
    zune_to_i420(data)
}

/// [`to_rgb`] with zune-jpeg.
pub fn zune_to_rgb(data: &[u8]) -> Result<Rgb8> {
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::RGB);
    let mut decoder = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(data), options);
    let pixels = decoder.decode().map_err(|e| anyhow::anyhow!("jpeg: {e:?}"))?;
    let info = decoder.info().context("jpeg: no header")?;
    samples_to_rgb(&pixels, 3, info.width as u32, info.height as u32)
}

/// [`to_i420`] with zune-jpeg, which upsamples chroma to full size first (interleaved YCbCr is its only YCbCr output).
pub fn zune_to_i420(data: &[u8]) -> Result<Option<VideoImage>> {
    let options = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::YCbCr);
    let mut decoder = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(data), options);
    decoder.decode_headers().map_err(|e| anyhow::anyhow!("jpeg: {e:?}"))?;
    let info = decoder.info().context("jpeg: no header")?;
    let (width, height) = (info.width as usize, info.height as usize);
    if decoder.input_colorspace() != Some(ColorSpace::YCbCr) || width == 0 || height == 0 || width % 2 == 1 || height % 2 == 1 {
        return Ok(None);
    }
    let pixels = decoder.decode().map_err(|e| anyhow::anyhow!("jpeg: {e:?}"))?;
    ensure!(pixels.len() >= width * height * 3, "jpeg: decoder returned too few samples");
    Ok(Some(VideoImage::i420(width as u32, height as u32, ycbcr_to_i420(&pixels, width, height))?))
}

/// Interleaved full-range YCbCr (JFIF, `width × height`, both even) as BT.601 limited-range I420, chroma from each
/// 2x2 block's mean.
pub fn ycbcr_to_i420(pixels: &[u8], width: usize, height: usize) -> Vec<u8> {
    let (luma_of, chroma_of) = (limited_luma, limited_chroma::<4>);
    let mut out = vec![0u8; width * height * 3 / 2];
    let (luma, chroma) = out.split_at_mut(width * height);
    let (u_plane, v_plane) = chroma.split_at_mut(width * height / 4);
    for (luma_value, pixel) in luma.iter_mut().zip(pixels.as_chunks::<3>().0) {
        *luma_value = luma_of(pixel[0]);
    }
    let half_width = width / 2;
    for (block_row, (u_row, v_row)) in u_plane.chunks_exact_mut(half_width).zip(v_plane.chunks_exact_mut(half_width)).enumerate() {
        let top = pixels[block_row * 2 * width * 3..][..width * 3].as_chunks::<6>().0;
        let bottom = pixels[(block_row * 2 + 1) * width * 3..][..width * 3].as_chunks::<6>().0;
        for ((u, v), (a, b)) in u_row.iter_mut().zip(v_row.iter_mut()).zip(top.iter().zip(bottom)) {
            *u = chroma_of(a[1] as u32 + a[4] as u32 + b[1] as u32 + b[4] as u32);
            *v = chroma_of(a[2] as u32 + a[5] as u32 + b[2] as u32 + b[5] as u32);
        }
    }
    out
}

/// A full-range JFIF luma sample as BT.601 limited range (16..235). Arithmetic rather than a lookup table: the compiler
/// vectorizes it (~1.4x faster on an Apple M core).
fn limited_luma(value: u8) -> u8 {
    (16 + (value as u32 * 219 + 127) / 255) as u8
}

/// The mean of `COUNT` full-range JFIF chroma samples (`sum`) as one BT.601 limited-range sample (16..240).
fn limited_chroma<const COUNT: u32>(sum: u32) -> u8 {
    (16 + (sum * 224 + 255 * COUNT / 2) / (255 * COUNT)) as u8
}

/// libjpeg-turbo: the video path takes the JPEG's own planes (`tj3DecompressToYUV8`, no upsampling), which for 4:2:0
/// are already I420 but for range.
#[cfg(feature = "turbojpeg")]
mod turbo {
    use super::*;
    use turbojpeg::{Colorspace, Decompressor, Image, PixelFormat, Subsamp, YuvImage};

    pub fn to_rgb(data: &[u8]) -> Result<Rgb8> {
        let mut decompressor = Decompressor::new()?;
        let header = decompressor.read_header(data).context("jpeg")?;
        if matches!(header.colorspace, Colorspace::CMYK | Colorspace::YCCK) {
            // TurboJPEG has no CMYK to RGB
            return zune_to_rgb(data);
        }
        let (width, height) = (header.width, header.height);
        let mut pixels = vec![0u8; width * height * 3];
        decompressor.decompress(data, Image { pixels: pixels.as_mut_slice(), width, pitch: width * 3, height, format: PixelFormat::RGB }).context("jpeg")?;
        Ok(Rgb8 { width: width as u32, height: height as u32, pixels })
    }

    pub fn to_i420(data: &[u8]) -> Result<Option<VideoImage>> {
        let mut decompressor = Decompressor::new()?;
        let header = decompressor.read_header(data).context("jpeg")?;
        let (width, height) = (header.width, header.height);
        if header.colorspace != Colorspace::YCbCr || width == 0 || height == 0 || width % 2 == 1 || height % 2 == 1 {
            return Ok(None);
        }
        // samples per I420 chroma sample, across and down
        let (across, down) = match header.subsamp {
            Subsamp::Sub2x2 => (1, 1),
            Subsamp::Sub2x1 => (1, 2),
            Subsamp::None => (2, 2),
            // 4:4:0, 4:1:1, 4:4:1: rare, and zune's full-size YCbCr already handles any sampling
            _ => return zune_to_i420(data),
        };
        let mut planes = YuvImage { pixels: vec![0u8; turbojpeg::yuv_pixels_len(width, 1, height, header.subsamp)?], width, align: 1, height, subsamp: header.subsamp };
        decompressor.decompress_to_yuv(data, planes.as_deref_mut()).context("jpeg")?;
        let (plane_width, plane_size) = (planes.uv_width(), planes.uv_width() * planes.uv_height());
        let mut pixels = planes.pixels;
        let (luma, chroma) = pixels.split_at_mut(width * height);
        for value in luma.iter_mut() {
            *value = limited_luma(*value);
        }
        if (across, down) == (1, 1) {
            for value in chroma.iter_mut() {
                *value = limited_chroma::<1>(*value as u32);
            }
            return Ok(Some(VideoImage::i420(width as u32, height as u32, pixels)?));
        }
        let mut out = Vec::with_capacity(width * height * 3 / 2);
        out.extend_from_slice(luma);
        for plane in [&chroma[..plane_size], &chroma[plane_size..][..plane_size]] {
            for rows in plane.chunks_exact(plane_width * down) {
                let (top, bottom) = rows.split_at(plane_width);
                if across == 1 {
                    out.extend(top.iter().zip(bottom).map(|(a, b)| limited_chroma::<2>(*a as u32 + *b as u32)));
                } else {
                    out.extend(top.as_chunks::<2>().0.iter().zip(bottom.as_chunks::<2>().0).map(|(a, b)| limited_chroma::<4>(a[0] as u32 + a[1] as u32 + b[0] as u32 + b[1] as u32)));
                }
            }
        }
        Ok(Some(VideoImage::i420(width as u32, height as u32, out)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// BT.601 limited-range I420 with chroma from each 2x2 block's mean, as zenoh-gateway converts RGB.
    fn rgb_to_i420(rgb: &Rgb8) -> Vec<u8> {
        let width = rgb.width as usize;
        let pixel = |x: usize, y: usize| {
            let at = (y * width + x) * 3;
            [0, 1, 2].map(|channel| rgb.pixels[at + channel] as i32)
        };
        let mut out: Vec<u8> = (0..rgb.pixels.len() / 3).map(|index| {
            let [r, g, b] = pixel(index % width, index / width);
            (((66 * r + 129 * g + 25 * b + 128) >> 8) + 16) as u8
        }).collect();
        for weights in [[-38, -74, 112], [112, -94, -18]] {
            for y in 0..rgb.height as usize / 2 {
                for x in 0..width / 2 {
                    let mean = |channel: usize| (pixel(2 * x, 2 * y)[channel] + pixel(2 * x + 1, 2 * y)[channel] + pixel(2 * x, 2 * y + 1)[channel] + pixel(2 * x + 1, 2 * y + 1)[channel] + 2) >> 2;
                    out.push((((weights[0] * mean(0) + weights[1] * mean(1) + weights[2] * mean(2) + 128) >> 8) + 128).clamp(0, 255) as u8);
                }
            }
        }
        out
    }

    #[test]
    fn jpeg_goes_to_i420_like_rgb_would() {
        let jpeg = include_bytes!("../../test/fixtures/test_image.jpg");
        let direct = to_i420(jpeg).unwrap().unwrap();
        assert_eq!(direct.format(), zenoh_gateway::PixelFormat::I420, "a YCbCr jpeg skips rgb");
        let rgb = to_rgb(jpeg).unwrap();
        let through_rgb = rgb_to_i420(&rgb);
        // RGB clips out-of-gamut YCbCr, so a few saturated pixels may differ more; on average they agree
        let mut differences: Vec<i32> = direct.data().iter().zip(&through_rgb).map(|(a, b)| (*a as i32 - *b as i32).abs()).collect();
        differences.sort();
        let mean = differences.iter().sum::<i32>() as f64 / differences.len() as f64;
        let p99 = differences[differences.len() * 99 / 100];
        assert!(mean < 1.0 && p99 <= 5, "I420 straight from the jpeg differs from going through RGB: mean {mean:.2}, p99 {p99}");
    }

    fn psnr(a: &[u8], b: &[u8]) -> f64 {
        assert_eq!(a.len(), b.len());
        let mse = a.iter().zip(b).map(|(x, y)| (*x as f64 - *y as f64).powi(2)).sum::<f64>() / a.len() as f64;
        10.0 * (255.0f64.powi(2) / mse.max(1e-9)).log10()
    }

    /// Smooth shading, edges and a little texture: what a camera sees, not a flat test card.
    fn scene(width: u32, height: u32) -> Rgb8 {
        let pixels = (0..height)
            .flat_map(|y| (0..width).map(move |x| (x, y)))
            .flat_map(|(x, y)| {
                let (fx, fy) = (x as f32 / width as f32, y as f32 / height as f32);
                let light = 0.6 + 0.4 * (fx * 3.0 + fy * 2.0).sin();
                let edge = if (x / 23 + y / 17) % 3 == 0 { 0.6 } else { 1.0 };
                [0.9f32, 0.6, 0.3].map(|tint| (255.0 * light * edge * (0.3 + 0.7 * (tint + fx * 0.5).fract())) as u8)
            })
            .collect();
        Rgb8 { width, height, pixels }
    }

    #[test]
    fn video_pictures_match_the_source_at_any_size_and_sampling() {
        use jpeg_encoder::SamplingFactor;
        for (width, height) in [(1920, 1080), (1362, 768), (640, 482), (101, 75), (64, 3), (2, 2)] {
            for sampling in [SamplingFactor::R_4_2_0, SamplingFactor::R_4_2_2, SamplingFactor::R_4_4_4, SamplingFactor::R_4_4_0] {
                let source = scene(width, height);
                let mut jpeg = Vec::new();
                let mut encoder = jpeg_encoder::Encoder::new(&mut jpeg, 95);
                encoder.set_sampling_factor(sampling);
                encoder.encode(&source.pixels, width as u16, height as u16, jpeg_encoder::ColorType::Rgb).unwrap();
                let picture = crate::utils::compressed_image::to_video(&jpeg, "jpeg").unwrap();
                let case = format!("{width}x{height} {sampling:?}");
                assert_eq!((picture.width(), picture.height()), (width, height), "{case}");
                if width % 2 == 1 || height % 2 == 1 {
                    assert_eq!(picture.format(), zenoh_gateway::PixelFormat::Rgb8, "{case}: odd sides decode to rgb");
                    let quality = psnr(picture.data(), &source.pixels);
                    assert!(quality > 30.0, "{case}: {quality:.1} dB against the source");
                    continue;
                }
                assert_eq!(picture.format(), zenoh_gateway::PixelFormat::I420, "{case}");
                if width * height < 64 {
                    // smaller than one block: its few chroma samples are mostly the encoder's edge padding, which
                    // libjpeg-turbo passes through and zune-jpeg's upsampling blends
                    continue;
                }
                let quality = psnr(picture.data(), &rgb_to_i420(&source));
                assert!(quality > 30.0, "{case}: {quality:.1} dB against the source");
                let agreement = psnr(picture.data(), &rgb_to_i420(&to_rgb(&jpeg).unwrap()));
                assert!(agreement > 40.0, "{case}: {agreement:.1} dB against the same jpeg through rgb");
                let decoders = psnr(picture.data(), zune_to_i420(&jpeg).unwrap().unwrap().data());
                assert!(decoders > 40.0, "{case}: {decoders:.1} dB against zune-jpeg's");
            }
        }
    }
}
