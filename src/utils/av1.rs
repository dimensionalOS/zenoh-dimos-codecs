//! AV1: reading a temporal unit's OBUs, and decoding one with rav1d.
//!
//! AV1 has no Annex-B start codes to scan for ([`bitstream::nal_units`](super::bitstream::nal_units));
//! a temporal unit is a run of OBUs that each carry their own size (the low-overhead bitstream format, spec 5.2), so
//! reading one is exact rather than a search. The sequence header OBU plays the part H.264's SPS/PPS play: only it says
//! how big the picture is, and a decoder can start only once it has one.
//!
//! The decoder is rav1d, the pure-Rust port of dav1d: an ordinary cargo dependency, with no system library to find at
//! runtime and no C toolchain to build (unlike libde265 for H.265). Its hand-written assembly is behind this crate's
//! `av1-asm` feature, because on x86 building it needs nasm.

use super::compressed_video::i420_from_planes;
use anyhow::{Context, Result, anyhow, bail, ensure};
use rav1d::include::dav1d::data::Dav1dData;
use rav1d::include::dav1d::dav1d::{Dav1dContext, Dav1dSettings};
use rav1d::include::dav1d::headers::DAV1D_PIXEL_LAYOUT_I420;
use rav1d::include::dav1d::picture::Dav1dPicture;
use rav1d::src::lib::{dav1d_close, dav1d_data_create, dav1d_default_settings, dav1d_get_picture, dav1d_open, dav1d_picture_unref, dav1d_send_data};
use std::mem::MaybeUninit;
use std::ptr::NonNull;
use zenoh_gateway::VideoImage;

/// OBU types this reads (spec 6.2.2); every other type is skipped by its size field.
pub(crate) const OBU_SEQUENCE_HEADER: u8 = 1;
/// A temporal delimiter separates temporal units in a stream. It carries nothing, and RTP's AV1 payload format says not
/// to send one (RFC 9180 §5), so [`temporal_unit`] drops it.
pub(crate) const OBU_TEMPORAL_DELIMITER: u8 = 2;
pub(crate) const OBU_FRAME_HEADER: u8 = 3;
pub(crate) const OBU_FRAME: u8 = 6;

/// `frame_type` in an uncompressed frame header (spec 6.8.2).
const KEY_FRAME: u8 = 0;

/// One OBU of a temporal unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Obu<'a> {
    /// `obu_type`
    pub(crate) kind: u8,
    /// the whole OBU, header and size field included: what a decoder or packetizer is handed
    pub(crate) bytes: &'a [u8],
    /// just the payload after the header and size field, which is what the spec's parsers read
    pub(crate) payload: &'a [u8],
}

/// One LEB128 value and the bytes it took (spec 4.10.5).
fn leb128(data: &[u8]) -> Result<(usize, usize)> {
    let mut value = 0usize;
    for index in 0..8 {
        let byte = *data.get(index).ok_or_else(truncated)?;
        value |= ((byte & 0x7f) as usize) << (index * 7);
        if byte & 0x80 == 0 {
            return Ok((value, index + 1));
        }
    }
    bail!("AV1 leb128 runs past 8 bytes")
}

fn truncated() -> anyhow::Error {
    anyhow!("AV1 temporal unit truncated")
}

/// The OBUs in a temporal unit, in order.
///
/// Every OBU has to carry its size: without one, an OBU runs to the end of the buffer, so a unit holding more than one
/// could not be split at all. Publishers write the low-overhead format (what SVT-AV1 and ffmpeg's `obu` muxer emit), so
/// a unit without size fields is rejected rather than guessed at.
pub(crate) fn obus(data: &[u8]) -> Result<Vec<Obu<'_>>> {
    let mut obus = Vec::new();
    let mut at = 0;
    while at < data.len() {
        let start = at;
        let header = *data.get(at).ok_or_else(truncated)?;
        at += 1;
        ensure!(header & 0x80 == 0, "AV1 OBU header has its forbidden bit set (not an OBU stream?)");
        let kind = (header >> 3) & 0xf;
        let (extension, has_size) = ((header >> 2) & 1 == 1, (header >> 1) & 1 == 1);
        if extension {
            at += 1;
        }
        ensure!(has_size, "AV1 OBU type {kind} has no size field: this takes the low-overhead bitstream format, where every OBU does");
        let (size, read) = leb128(data.get(at..).ok_or_else(truncated)?)?;
        at += read;
        let payload = data.get(at..at + size).ok_or_else(truncated)?;
        at += size;
        obus.push(Obu { kind, bytes: &data[start..at], payload });
    }
    ensure!(!obus.is_empty(), "AV1 temporal unit is empty");
    Ok(obus)
}

/// Whether an OBU holds a key frame a decoder can start at, given whether the sequence header said
/// `reduced_still_picture_header` (with it there is no frame header to read at all: the frame is a shown key frame,
/// spec 5.9.2).
fn is_shown_keyframe(obu: &Obu<'_>, reduced_still_picture: bool) -> Result<bool> {
    if !matches!(obu.kind, OBU_FRAME | OBU_FRAME_HEADER) {
        return Ok(false);
    }
    if reduced_still_picture {
        return Ok(true);
    }
    // show_existing_frame f(1), frame_type f(2), show_frame f(1)
    let bits = *obu.payload.first().ok_or_else(truncated)?;
    let (show_existing, frame_type, show_frame) = (bits >> 7, (bits >> 5) & 3, (bits >> 4) & 1);
    Ok(show_existing == 0 && frame_type == KEY_FRAME && show_frame == 1)
}

/// A temporal unit read for a passthrough track: the bytes to forward, and what can be told about it without decoding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct TemporalUnit {
    /// every OBU type in the unit, in order, for the error messages
    pub(crate) types: Vec<u8>,
    /// the OBUs to forward: the unit without its temporal delimiter, which RTP does not carry
    pub(crate) data: Vec<u8>,
    /// the sequence header OBU, when the unit carries one: the AV1 equivalent of H.264's SPS/PPS, to be put back in
    /// front of a keyframe that arrives without it
    pub(crate) sequence_header: Option<Vec<u8>>,
    /// a shown key frame, so a decoder with a sequence header can start here
    pub(crate) keyframe: bool,
    /// the picture size, when the unit carries a sequence header
    pub(crate) size: Option<(u32, u32)>,
}

/// Reads a temporal unit for forwarding.
pub(crate) fn temporal_unit(data: &[u8]) -> Result<TemporalUnit> {
    let obus = obus(data)?;
    // the sequence header has to be read first: whether a frame OBU has a header to read at all depends on it
    let header = obus.iter().find(|obu| obu.kind == OBU_SEQUENCE_HEADER);
    let reduced_still_picture = match header {
        // seq_profile f(3), still_picture f(1), reduced_still_picture_header f(1)
        Some(obu) => (*obu.payload.first().ok_or_else(truncated)? >> 3) & 1 == 1,
        None => false,
    };
    let size = header.map(|obu| super::av1_seq::picture_size(obu.payload)).transpose()?;
    let mut keyframe = false;
    let mut forwarded = Vec::with_capacity(data.len());
    for obu in &obus {
        keyframe |= is_shown_keyframe(obu, reduced_still_picture)?;
        if obu.kind != OBU_TEMPORAL_DELIMITER {
            forwarded.extend_from_slice(obu.bytes);
        }
    }
    ensure!(!forwarded.is_empty(), "AV1 temporal unit holds nothing but a temporal delimiter");
    Ok(TemporalUnit {
        types: obus.iter().map(|obu| obu.kind).collect(),
        data: forwarded,
        sequence_header: header.map(|obu| obu.bytes.to_vec()),
        keyframe,
        size,
    })
}

/// The first picture in a temporal unit, as I420. Each sample is decoded by a decoder of its own, so the unit has to
/// carry its sequence header and a key frame;
/// [`bitstream::check_decodable_alone`](crate::utils::bitstream::check_decodable_alone) is what says so first, in the
/// same words for every codec.
pub fn decode(data: &[u8]) -> Result<VideoImage> {
    let mut decoder = Decoder::open()?;
    decoder.send(data)?;
    decoder.picture()
}

/// A decoder that keeps its state between temporal units, so **predicted frames decode**.
///
/// [`decode`] opens a decoder per call, which is right when each sample is sent on its own but means only a key frame
/// ever produces a picture. A `Stream` is fed every temporal unit of one stream, in order, and gives back each picture
/// as it completes — the same shape as [`h264::Stream`](super::h264::Stream) and [`h265::Stream`](super::h265::Stream).
///
/// rav1d needs no system library and no C toolchain, so unlike the H.265 path this works in a static musl build and on
/// a machine with nothing installed. Its hand-written assembly (worth several times the speed) is behind this crate's
/// `av1-asm` feature, off by default because on x86 building it needs nasm and a build dependency nobody asked for is
/// exactly what must not be imposed on a downstream crate.
pub struct Stream {
    decoder: Decoder,
    /// skipping until the next key frame: nothing decodable has been fed yet, or samples were lost
    resyncing: bool,
}

// SAFETY: an rav1d context is not thread safe, and `Stream` owns its decoder and lends it to nobody, so the context is
// only ever touched through `&mut self`. The gateway hands a stream decoder from one blocking task to the next, never
// to two at once, which is what `Send` (and not `Sync`) says.
unsafe impl Send for Stream {}

impl Stream {
    /// A decoder waiting for its first key frame.
    pub fn new() -> Result<Self> {
        Ok(Stream { decoder: Decoder::open()?, resyncing: true })
    }

    /// Start over at the next key frame, because samples before it were lost.
    ///
    /// The decoder itself is kept, as on the other two paths: a key frame has no references, so it makes the picture
    /// whole by itself, and that is the whole of what a new decoder would give.
    pub fn resync(&mut self) {
        self.resyncing = true;
    }

    /// Feeds one temporal unit, which follows every unit fed before it, and returns the picture it completed.
    ///
    /// `keyframe`: the unit is decodable on its own (a shown key frame with its sequence header). While resyncing,
    /// anything else is dropped unread and this gives `Ok(None)`.
    pub fn push(&mut self, data: &[u8], keyframe: bool) -> Result<Option<VideoImage>> {
        if self.resyncing {
            if !keyframe {
                return Ok(None);
            }
            self.resyncing = false;
        }
        self.decoder.send(data)?;
        self.decoder.try_picture()
    }
}

/// An open rav1d decoder, closed on drop.
struct Decoder {
    context: Option<Dav1dContext>,
}

impl Decoder {
    fn open() -> Result<Self> {
        let mut settings = MaybeUninit::<Dav1dSettings>::uninit();
        // SAFETY: dav1d_default_settings writes a whole Dav1dSettings through the pointer and reads nothing.
        unsafe { dav1d_default_settings(NonNull::new(settings.as_mut_ptr()).expect("a stack slot is never null")) };
        // SAFETY: dav1d_default_settings initialised every field.
        let mut settings = unsafe { settings.assume_init() };
        // one picture out for each one in, on this thread: a sample is a single frame, so there is nothing to pipeline
        // and a frame delay would mean sending and then flushing for every one
        settings.n_threads = 1;
        settings.max_frame_delay = 1;
        let mut context: Option<Dav1dContext> = None;
        // SAFETY: both pointers are to live locals, which is all dav1d_open asks.
        let result = unsafe { dav1d_open(NonNull::new(&mut context), NonNull::new(&mut settings)) };
        ensure!(result.0 == 0 && context.is_some(), "rav1d could not open a decoder ({})", result.0);
        Ok(Decoder { context })
    }

    fn send(&mut self, data: &[u8]) -> Result<()> {
        let mut buffer = Dav1dData::default();
        // SAFETY: `buffer` is a live local; the call either fills it and returns its data, or returns null.
        let into = unsafe { dav1d_data_create(NonNull::new(&mut buffer), data.len()) };
        ensure!(!into.is_null(), "rav1d could not take {} bytes of AV1", data.len());
        // SAFETY: dav1d_data_create just returned `data.len()` writable bytes at `into`, which `data` cannot overlap.
        unsafe { std::ptr::copy_nonoverlapping(data.as_ptr(), into, data.len()) };
        // SAFETY: the context is open and `buffer` is a live local holding the data just written.
        let result = unsafe { dav1d_send_data(self.context, NonNull::new(&mut buffer)) };
        ensure!(result.0 == 0, "rav1d rejected the AV1 temporal unit ({})", result.0);
        Ok(())
    }

    /// The picture that is ready, or `None` because the decoder wants more data first.
    ///
    /// `EAGAIN` is not an error here: a decoder that holds a frame answers it for every unit until one comes out, and
    /// treating it as a failure would make a working stream look broken (and, on the per-sample path, would hide the
    /// real reason a sample produced nothing).
    fn try_picture(&mut self) -> Result<Option<VideoImage>> {
        let mut picture = Dav1dPicture::default();
        // SAFETY: the context is open and `picture` is a live, zeroed local, as dav1d_get_picture wants.
        let result = unsafe { dav1d_get_picture(self.context, NonNull::new(&mut picture)) };
        // owned from here, so an early return below still unreferences it
        let picture = Picture(picture);
        // `libc` is a Linux-only dependency here (core pinning), and EAGAIN is 11 on Linux and 35 on macOS, so the
        // errno is read through std, which knows each platform's number
        if std::io::Error::from_raw_os_error(-result.0).kind() == std::io::ErrorKind::WouldBlock {
            return Ok(None);
        }
        ensure!(result.0 == 0, "rav1d could not give a picture for the AV1 temporal unit ({})", result.0);
        Self::read(&picture).map(Some)
    }

    /// The picture a unit must have produced, or why it did not.
    fn picture(&mut self) -> Result<VideoImage> {
        self.try_picture()?.context("rav1d decoded no picture from the AV1 temporal unit")
    }

    /// One of rav1d's pictures as an I420 [`VideoImage`].
    fn read(picture: &Picture) -> Result<VideoImage> {
        let parameters = &picture.0.p;
        ensure!(parameters.layout == DAV1D_PIXEL_LAYOUT_I420, "AV1 picture is not 4:2:0 (layout {})", parameters.layout);
        ensure!(parameters.bpc == 8, "AV1 picture is {}-bit, and the video path takes 8-bit I420", parameters.bpc);
        let (width, height) = (usize::try_from(parameters.w)?, usize::try_from(parameters.h)?);
        let (y_stride, uv_stride) = (usize::try_from(picture.0.stride[0])?, usize::try_from(picture.0.stride[1])?);
        let plane = |index: usize, stride: usize, rows: usize| -> Result<&[u8]> {
            let data = picture.0.data[index].ok_or_else(|| anyhow!("AV1 picture has no plane {index}"))?;
            // SAFETY: rav1d owns these planes until `picture` is dropped, and each holds `stride * rows` bytes.
            Ok(unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, stride * rows) })
        };
        let chroma_rows = height.div_ceil(2);
        i420_from_planes(width, height, [(plane(0, y_stride, height)?, y_stride), (plane(1, uv_stride, chroma_rows)?, uv_stride), (plane(2, uv_stride, chroma_rows)?, uv_stride)])
    }
}

impl Drop for Decoder {
    fn drop(&mut self) {
        // SAFETY: `context` is the one dav1d_open filled, and this runs once.
        unsafe { dav1d_close(NonNull::new(&mut self.context)) };
    }
}

/// A decoded picture, unreferenced on drop: it owns the planes read out of it.
struct Picture(Dav1dPicture);

impl Drop for Picture {
    fn drop(&mut self) {
        // SAFETY: `self.0` is a live local, either the picture dav1d_get_picture filled or the zeroed one it left.
        unsafe { dav1d_picture_unref(NonNull::new(&mut self.0)) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    #[test]
    fn decodes_a_keyframe() {
        crate::utils::compressed_video::tests::assert_like_pattern(&decode(&fixture("compressed/pattern.av1")).unwrap(), "av1 keyframe");
    }

    #[test]
    fn reads_a_temporal_unit() {
        // temporal delimiter, sequence header, frame
        let unit = temporal_unit(&fixture("compressed/pattern.av1")).unwrap();
        assert_eq!(unit.types, vec![OBU_TEMPORAL_DELIMITER, OBU_SEQUENCE_HEADER, OBU_FRAME]);
        assert_eq!((unit.keyframe, unit.size), (true, Some((320, 240))));
        assert!(unit.sequence_header.is_some());
        // the temporal delimiter is dropped, and it is the only thing dropped
        let whole = fixture("compressed/pattern.av1");
        assert_eq!(unit.data.len(), whole.len() - obus(&whole).unwrap()[0].bytes.len());
    }

    #[test]
    fn a_unit_without_a_sequence_header_has_no_size() {
        let unit = temporal_unit(&fixture("compressed/pattern_bare.av1")).unwrap();
        assert_eq!((unit.keyframe, unit.size, unit.sequence_header), (true, None, None));
        // what the per-sample path makes of that is `bitstream`'s to say, in the same words it uses for H.26x
    }

    #[test]
    fn rejects_what_is_not_an_obu_stream() {
        // Annex-B H.264 starts 00 00 00 01: the first byte reads as an OBU header with no size field
        let annex_b = obus(&[0, 0, 0, 1, 0x67, 0x64]).unwrap_err().to_string();
        assert!(annex_b.contains("no size field"), "{annex_b}");
        let forbidden = obus(&[0x80]).unwrap_err().to_string();
        assert!(forbidden.contains("forbidden bit"), "{forbidden}");
        // a size field that runs past the end
        let short = obus(&[0x12, 0x40, 1, 2, 3]).unwrap_err().to_string();
        assert!(short.contains("truncated"), "{short}");
        assert!(obus(&[]).unwrap_err().to_string().contains("empty"));
    }

    /// The temporal units of a stream written as one buffer, split where each temporal delimiter starts one.
    ///
    /// Test-only: a publisher sends one unit per sample, which is what [`temporal_unit`] takes. An encoder writing to a
    /// file concatenates them, so a fixture of a real stream has to be cut back into samples to be fed as one.
    fn temporal_units(data: &[u8]) -> Vec<Vec<u8>> {
        let mut units: Vec<Vec<u8>> = Vec::new();
        for obu in obus(data).unwrap() {
            if obu.kind == OBU_TEMPORAL_DELIMITER || units.is_empty() {
                units.push(Vec::new());
            }
            units.last_mut().expect("just pushed").extend_from_slice(obu.bytes);
        }
        units
    }

    /// The fixture is what the tests below rest on, so what it *is* gets asserted rather than assumed: twelve samples,
    /// the first a key frame carrying its sequence header and the other eleven predicted from it.
    #[test]
    fn the_predicted_fixture_is_one_keyframe_and_eleven_inter_frames() {
        let units = temporal_units(&fixture("compressed/pattern_predicted.av1"));
        assert_eq!(units.len(), 12);
        let first = temporal_unit(&units[0]).unwrap();
        assert_eq!((first.keyframe, first.size), (true, Some((320, 240))));
        assert!(first.sequence_header.is_some());
        for (index, unit) in units.iter().enumerate().skip(1) {
            let read = temporal_unit(unit).unwrap();
            assert!(!read.keyframe, "unit {index} is predicted");
            assert_eq!((read.size, read.sequence_header), (None, None), "unit {index} carries no sequence header");
        }
    }

    /// The whole reason a stateful decoder exists: every predicted frame decodes, where the per-sample path gets one
    /// picture in twelve. The pictures are checked against the pattern too — a decoder that produced *something* for
    /// each unit while losing the reference chain would otherwise pass.
    #[test]
    fn a_stream_decodes_the_predicted_frames_a_per_sample_decode_cannot() {
        let units = temporal_units(&fixture("compressed/pattern_predicted.av1"));
        // alone, only the first unit decodes at all
        crate::utils::compressed_video::tests::assert_like_pattern(&decode(&units[0]).unwrap(), "the key frame alone");
        assert!(decode(&units[1]).is_err(), "a predicted frame cannot be decoded on its own");

        let mut stream = Stream::new().unwrap();
        let mut pictures = 0;
        for (index, unit) in units.iter().enumerate() {
            let read = temporal_unit(unit).unwrap();
            if let Some(picture) = stream.push(unit, read.keyframe).unwrap() {
                crate::utils::compressed_video::tests::assert_like_pattern(&picture, &format!("stream picture {index}"));
                pictures += 1;
            }
        }
        assert_eq!(pictures, units.len(), "every unit of the stream produced its picture");
    }

    /// After a gap the decoder must show nothing until the next key frame, rather than a picture built on frames it
    /// never saw — and it must not need a *new* decoder to do it.
    #[test]
    fn a_stream_waits_for_a_key_frame_before_and_after_a_gap() {
        let units = temporal_units(&fixture("compressed/pattern_predicted.av1"));
        let mut stream = Stream::new().unwrap();
        // before any key frame there is nothing to predict from
        assert!(stream.push(&units[3], false).unwrap().is_none());
        assert!(stream.push(&units[0], true).unwrap().is_some(), "the key frame starts it");
        assert!(stream.push(&units[1], false).unwrap().is_some());
        stream.resync();
        assert!(stream.push(&units[2], false).unwrap().is_none(), "nothing after a gap until a key frame");
        assert!(stream.push(&units[0], true).unwrap().is_some(), "and the next key frame makes it whole");
    }

    #[test]
    fn leb128_reads_multi_byte_values() {
        assert_eq!(leb128(&[0x00]).unwrap(), (0, 1));
        assert_eq!(leb128(&[0x7f]).unwrap(), (127, 1));
        assert_eq!(leb128(&[0x80, 0x01]).unwrap(), (128, 2));
        assert!(leb128(&[0x80; 9]).is_err());
    }
}
