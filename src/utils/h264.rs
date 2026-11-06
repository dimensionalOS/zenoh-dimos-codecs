//! H.264 decoding with openh264 (the C sources zenoh-gateway's encoder already builds).

use super::compressed_video::i420_from_planes;
use anyhow::{Context, Result};
use openh264::decoder::Decoder;
use openh264::formats::YUVSource;
use zenoh_gateway::VideoImage;

/// The first picture in an Annex-B access unit (with its parameter sets), as I420.
pub fn decode(data: &[u8]) -> Result<VideoImage> {
    let mut decoder = Decoder::new().context("openh264 decoder")?;
    let picture = decoder.decode(data).context("openh264")?;
    let to_image = |yuv: &dyn YUVSource| {
        let ((width, height), (y_stride, u_stride, v_stride)) = (yuv.dimensions(), yuv.strides());
        i420_from_planes(width, height, [(yuv.y(), y_stride), (yuv.u(), u_stride), (yuv.v(), v_stride)])
    };
    if let Some(yuv) = picture {
        return to_image(&yuv);
    }
    // a decoder that holds the picture back gives it up on flush
    let flushed = decoder.flush_remaining().context("openh264 flush")?;
    to_image(flushed.first().context("openh264 decoded no picture")?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn decodes_a_keyframe() {
        crate::utils::compressed_video::tests::assert_like_pattern(&decode(&fixture("compressed/pattern.h264")).unwrap(), "h264 keyframe");
    }
}
