//! Raw `sensor_msgs/Image` pixels (rgb8, bgr8, rgba8, bgra8, mono8, mono16) to packed RGB8 for video.

use crate::codecs::msgs::RawImage;
use crate::utils::compressed_image;
use anyhow::{Context, Result, bail, ensure};
use zenoh_gateway::VideoImage;

/// Packed 8-bit RGB, row-major, no padding.
pub struct Rgb8 {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl Rgb8 {
    pub fn into_video(self) -> Result<VideoImage> {
        VideoImage::rgb8(self.width, self.height, self.pixels)
    }
}

pub fn row<'a>(image: &RawImage<'a>, y: u32, bytes_per_pixel: usize) -> Result<&'a [u8]> {
    let width_bytes = image.width as usize * bytes_per_pixel;
    let step = if image.step == 0 { width_bytes } else { image.step as usize };
    ensure!(step >= width_bytes, "step {step} < width {} x {bytes_per_pixel} bytes", image.width);
    let start = y as usize * step;
    image.data.get(start..start + width_bytes).with_context(|| format!("image data too short for row {y} ({} bytes)", image.data.len()))
}

pub fn read_u16(bytes: &[u8], big_endian: bool) -> u16 {
    if big_endian { u16::from_be_bytes([bytes[0], bytes[1]]) } else { u16::from_le_bytes([bytes[0], bytes[1]]) }
}

/// Writes one pixel's RGB from its source bytes (and the message's big-endian flag).
type PixelConverter = fn(&[u8], bool, &mut [u8]);

/// Image `encoding`s that are a file in the data field (dimos' jpeg-encoded Image), not raw pixels.
const FILE_ENCODINGS: [&str; 5] = ["jpeg", "jpg", "png", "webp", "jxl"];

/// A raw image (or a dimos Image carrying a jpeg/png file) as RGB8. 16-bit gray keeps its top 8 bits.
pub fn to_rgb(image: &RawImage) -> Result<Rgb8> {
    let encoding = image.encoding.to_ascii_lowercase();
    if FILE_ENCODINGS.contains(&encoding.as_str()) {
        return compressed_image::to_rgb(image.data, &encoding);
    }
    let (bytes_per_pixel, convert): (usize, PixelConverter) = match encoding.as_str() {
        "rgb8" => (3, |s, _, d| d.copy_from_slice(&s[..3])),
        "bgr8" | "8uc3" => (3, |s, _, d| d.copy_from_slice(&[s[2], s[1], s[0]])),
        "rgba8" => (4, |s, _, d| d.copy_from_slice(&s[..3])),
        "bgra8" | "8uc4" => (4, |s, _, d| d.copy_from_slice(&[s[2], s[1], s[0]])),
        "mono8" | "8uc1" => (1, |s, _, d| d.fill(s[0])),
        "mono16" | "16uc1" => (2, |s, big, d| d.fill((read_u16(s, big) >> 8) as u8)),
        other => bail!("image encoding {other:?} has no color conversion (supported: rgb8 bgr8 rgba8 bgra8 mono8 mono16 16UC1 jpeg png webp jxl)"),
    };
    ensure!(image.width > 0 && image.height > 0, "empty image");
    let mut pixels = vec![0u8; image.width as usize * image.height as usize * 3];
    for y in 0..image.height {
        let source = row(image, y, bytes_per_pixel)?;
        let destination = &mut pixels[y as usize * image.width as usize * 3..][..image.width as usize * 3];
        for (source_pixel, destination_pixel) in source.chunks_exact(bytes_per_pixel).zip(destination.as_chunks_mut::<3>().0.iter_mut()) {
            convert(source_pixel, image.big_endian, destination_pixel);
        }
    }
    Ok(Rgb8 { width: image.width, height: image.height, pixels })
}

/// Expands decoded samples of `channels` per pixel (1 gray, 2 gray+alpha, 3 rgb, 4 rgba) to RGB8.
pub fn samples_to_rgb(samples: &[u8], channels: usize, width: u32, height: u32) -> Result<Rgb8> {
    ensure!(samples.len() >= width as usize * height as usize * channels, "decoder returned too few samples");
    let pixels = match channels {
        3 => samples[..width as usize * height as usize * 3].to_vec(),
        1 | 2 => samples.chunks_exact(channels).flat_map(|p| [p[0], p[0], p[0]]).collect(),
        4 => samples.as_chunks::<4>().0.iter().flat_map(|p| [p[0], p[1], p[2]]).collect(),
        other => bail!("{other} channels per pixel"),
    };
    Ok(Rgb8 { width, height, pixels })
}

/// A raw image for the video path: like [`to_rgb`], except a JPEG goes straight to I420
/// (see [`compressed_image::to_video`]).
pub fn to_video(image: &RawImage) -> Result<VideoImage> {
    let encoding = image.encoding.to_ascii_lowercase();
    if FILE_ENCODINGS.contains(&encoding.as_str()) {
        return compressed_image::to_video(image.data, &encoding);
    }
    to_rgb(image)?.into_video()
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use crate::codecs::msgs::{dimos_lcm_image, ros2_image};
    use crate::utils::tests::fixture;

    pub fn quadrant_means(rgb: &Rgb8) -> [[f64; 3]; 4] {
        let mut sums = [[0f64; 3]; 4];
        let mut counts = [0f64; 4];
        let (half_width, half_height) = (rgb.width / 2, rgb.height / 2);
        for y in 0..rgb.height {
            for x in 0..rgb.width {
                let quadrant = (if y < half_height { 0 } else { 2 }) + (if x < half_width { 0 } else { 1 });
                let pixel = &rgb.pixels[(y * rgb.width + x) as usize * 3..][..3];
                for channel in 0..3 {
                    sums[quadrant][channel] += pixel[channel] as f64;
                }
                counts[quadrant] += 1.0;
            }
        }
        std::array::from_fn(|quadrant| sums[quadrant].map(|sum| sum / counts[quadrant]))
    }

    const PATTERN: [[f64; 3]; 4] = [[255.0, 0.0, 0.0], [0.0, 255.0, 0.0], [0.0, 0.0, 255.0], [255.0, 255.0, 255.0]];

    pub fn assert_pattern(rgb: &Rgb8, tolerance: f64, what: &str) {
        assert_eq!((rgb.width, rgb.height), (320, 240), "{what}");
        let means = quadrant_means(rgb);
        for (quadrant, expected) in PATTERN.iter().enumerate() {
            for channel in 0..3 {
                assert!((means[quadrant][channel] - expected[channel]).abs() <= tolerance, "{what}: quadrant {quadrant} = {:?}", means[quadrant]);
            }
        }
    }

    #[test]
    fn raw_color_encodings() {
        for file in ["ros2/image_rgb8.cdr", "ros2/image_bgr8.cdr"] {
            assert_pattern(&to_rgb(&ros2_image::parse(&fixture(file)).unwrap()).unwrap(), 0.0, file);
        }
        for file in ["dimos/image_rgb8.bin", "dimos/image_bgr8.bin"] {
            assert_pattern(&to_rgb(&dimos_lcm_image::parse(&fixture(file)).unwrap()).unwrap(), 0.0, file);
        }
        assert_pattern(&to_rgb(&dimos_lcm_image::parse(&fixture("dimos/image_jpeg_in_Image.bin")).unwrap()).unwrap(), 8.0, "jpeg in Image");
        let mono = to_rgb(&ros2_image::parse(&fixture("ros2/image_mono16.cdr")).unwrap()).unwrap();
        assert_eq!(quadrant_means(&mono).map(|m| m[0]), [0.0, 85.0, 170.0, 255.0]);
    }
}
