//! The **per-sample** path for `foxglove_msgs/CompressedVideo` data: one coded picture to a picture for the video
//! encoder, decoded with a decoder of its own.
//!
//! Each sample is decoded alone (the gateway decodes only the samples it sends and shares the result between
//! frontends), so a sample must be a keyframe carrying the bytes a decoder starts from: a stream with predicted frames,
//! or one whose headers are sent once, can't play this way — about one frame a second on a camera with a keyframe a
//! second. That is what [`video_stream`](super::video_stream) exists to fix, and this path remains for the encodings
//! and channels that do not need it.
//!
//! Reading the bytes themselves is [`bitstream`](super::bitstream)'s, for all three codecs; what is left here is the
//! per-sample rule and the shared I420 plane copy the decoders hand their pictures over with.

use super::bitstream;
use anyhow::{Result, ensure};
use zenoh_gateway::VideoImage;

/// The picture in one coded unit of `format` (`h264`, `h265`, `av1`).
pub fn to_video(data: &[u8], format: &str) -> Result<VideoImage> {
    let bitstream = bitstream::by_name(format)?;
    bitstream::check_decodable_alone(bitstream, data)?;
    bitstream.decode_alone(data)
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

    /// Every codec's own keyframe decodes through the one call, and the picture is the picture — the per-sample path
    /// reaching all three is what says `decode_alone` is wired to the right decoder in each impl.
    #[test]
    fn a_keyframe_of_any_codec_decodes_on_its_own() {
        for (format, file) in [("h264", "compressed/pattern.h264"), ("h265", "compressed/pattern.h265"), ("av1", "compressed/pattern.av1")] {
            match to_video(&fixture(file), format) {
                Ok(picture) => assert_like_pattern(&picture, format),
                // libde265 is loaded with dlopen and is not on every machine; the others are built in
                Err(error) if format == "h265" && error.to_string().contains("libde265") => eprintln!("skipping h265: {error:#}"),
                Err(error) => panic!("{format}: {error:#}"),
            }
        }
    }

    /// What this path cannot take, said in the publisher's terms rather than as a decoder error: a predicted frame, a
    /// keyframe without its headers, and a format nothing here reads.
    #[test]
    fn a_sample_must_be_a_keyframe_with_the_bytes_a_decoder_starts_from() {
        // an H.265 trailing picture (TRAIL_R, type 1) is predicted
        let predicted = to_video(&[0, 0, 0, 1, 0x02, 0x01, 0xaa], "h265").unwrap_err().to_string();
        assert!(predicted.contains("isn't a keyframe"), "{predicted}");
        let bare = to_video(&fixture("compressed/pattern_bare.h265"), "h265").unwrap_err().to_string();
        assert!(bare.contains("no VPS/SPS/PPS"), "{bare}");
        let bare_av1 = to_video(&fixture("compressed/pattern_bare.av1"), "av1").unwrap_err().to_string();
        assert!(bare_av1.contains("no sequence header OBU"), "{bare_av1}");
        assert!(to_video(b"no start code", "h264").is_err());
        assert!(to_video(&fixture("compressed/pattern.h264"), "vp9").is_err());
    }

    #[test]
    fn planes_with_strides_crop_to_even_sizes() {
        // 3x3 with stride 4 (chroma 2x2 with stride 2): crops to 2x2
        let luma = [1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 0];
        let image = i420_from_planes(3, 3, [(&luma, 4), (&[10, 11, 12, 13], 2), (&[20, 21, 22, 23], 2)]).unwrap();
        assert_eq!((image.width(), image.height(), image.data()), (2, 2, &[1, 2, 4, 5, 10, 20][..]));
    }
}
