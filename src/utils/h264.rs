//! H.264 decoding with openh264 (the C sources zenoh-gateway's encoder already builds).

use super::compressed_video::i420_from_planes;
use anyhow::{Context, Result};
use openh264::decoder::Decoder;
use openh264::formats::YUVSource;
use zenoh_gateway::VideoImage;

/// One of openh264's pictures as an I420 [`VideoImage`].
fn to_image(yuv: &dyn YUVSource) -> Result<VideoImage> {
    let ((width, height), (y_stride, u_stride, v_stride)) = (yuv.dimensions(), yuv.strides());
    i420_from_planes(width, height, [(yuv.y(), y_stride), (yuv.u(), u_stride), (yuv.v(), v_stride)])
}

/// The first picture in an Annex-B access unit (with its parameter sets), as I420.
pub fn decode(data: &[u8]) -> Result<VideoImage> {
    let mut decoder = Decoder::new().context("openh264 decoder")?;
    let picture = decoder.decode(data).context("openh264")?;
    if let Some(yuv) = picture {
        return to_image(&yuv);
    }
    // a decoder that holds the picture back gives it up on flush
    let flushed = decoder.flush_remaining().context("openh264 flush")?;
    to_image(flushed.first().context("openh264 decoded no picture")?)
}

/// A decoder that keeps its state between access units, so **predicted frames decode**.
///
/// [`decode`] makes a fresh decoder per call, which is right when each sample is sent on its own but means only a
/// keyframe ever produces a picture. A `Stream` is fed every access unit of one stream, in order, and gives back each
/// picture as it completes. The same shape as [`h265::Stream`](super::h265::Stream) and
/// [`av1::Stream`](super::av1::Stream), so [`decoders`](crate::decoders) can treat all three alike.
///
/// openh264 needs no system library and no toolchain beyond the C compiler the gateway's encoder already uses, so an
/// H.264 source gets a stateful decoder on every machine — including a static musl build, where a dlopen'd library
/// (libde265) cannot be found at all.
pub struct Stream {
    decoder: Decoder,
    /// skipping until the next keyframe: nothing decodable has been fed yet, or samples were lost
    resyncing: bool,
}

impl Stream {
    /// A decoder waiting for its first keyframe.
    pub fn new() -> Result<Self> {
        Ok(Stream { decoder: Decoder::new().context("openh264 decoder")?, resyncing: true })
    }

    /// Start over at the next keyframe, because samples before it were lost.
    ///
    /// The decoder itself is kept. An IDR picture clears the reference pictures by definition, which is the whole of
    /// what a new decoder would give, and building one per lost sample is a stall per gap on a decoder that was working
    /// (~35 ms each where that was measured, on the GStreamer path).
    pub fn resync(&mut self) {
        self.resyncing = true;
    }

    /// Feeds one Annex-B access unit, which follows every unit fed before it, and returns the picture it completed.
    ///
    /// `keyframe`: the unit is decodable on its own (an IDR with its parameter sets). While resyncing, anything else is
    /// dropped unread and this gives `Ok(None)`.
    pub fn push(&mut self, data: &[u8], keyframe: bool) -> Result<Option<VideoImage>> {
        if self.resyncing {
            if !keyframe {
                return Ok(None);
            }
            self.resyncing = false;
        }
        // nothing is flushed here, unlike `decode`: a flush would give up the frame the decoder is holding and so
        // break the reference chain the next unit needs. A picture held back comes out of a later push, which is what
        // `last_frame_id` upstream exists to pair correctly.
        match self.decoder.decode(data).context("openh264")? {
            Some(yuv) => to_image(&yuv).map(Some),
            None => Ok(None),
        }
    }
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
