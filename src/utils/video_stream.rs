//! The stateful side of compressed video: a [`StreamDecoder`] that keeps one H.265 decoder per stream, so predicted
//! frames decode and the gateway can re-encode all of them.
//!
//! This is the path a browser with no HEVC decoder takes (Firefox, which has no H.265 over WebRTC). Such a browser
//! cannot be sent the publisher's bitstream, so the gateway has to decode it and encode it again — and
//! [`compressed_video::to_video`](super::compressed_video::to_video) decodes each sample on its own, which only a
//! keyframe can be. On this camera that is one frame in thirty.
//!
//! A decoder that keeps its state fixes that but changes what it needs: every sample of the stream, in arrival order,
//! with nothing skipped. The gateway arranges that for a `StreamDecoder` — it stops thinning the stream, decodes one
//! sample at a time, and shares the frames with nobody, since another subscription's decoder sits at another point in
//! the stream. When samples are lost anyway, it calls [`resync`](StreamDecoder::resync) and this skips to the next
//! keyframe rather than showing a picture built on frames it never saw.

use super::compressed_video::{self, Codec};
use super::h265;
use crate::codecs::msgs::{CompressedVideo, Header};
use anyhow::{Context, Result, bail};
use std::collections::VecDeque;
use zenoh_gateway::{DecodedFrame, EncodingSample, StreamDecoder, VideoImage};

/// How a compressed-video encoding reads its own wire format (`ros2_compressed_video`'s CDR,
/// `dimos_lcm_compressed_video`'s LCM), so one decoder serves both.
pub type Parse = for<'a> fn(&'a [u8]) -> Result<CompressedVideo<'a>>;

/// Which decoder a stream ended up with, since it depends on what is installed.
enum Backend {
    /// GStreamer, by default libav's `avdec_h265`: ~91 fps of 720p on an RK3588 A76 core
    #[cfg(feature = "gstreamer")]
    Gstreamer(crate::encoders::gstreamer::H265Decoder),
    /// libde265, loaded with dlopen: ~50 fps on the same core, and the only one where GStreamer is not installed
    Libde265(h265::Stream),
}

impl Backend {
    /// The fastest decoder this machine has. GStreamer first: on the hardware this runs on, libde265 is not fast
    /// enough for one camera at 30 Hz, let alone two, and `avdec_h265` is nearly twice as quick. libde265 stays as the
    /// fallback because it needs only a shared library, and it is what a developer machine without GStreamer gets.
    ///
    /// `ZDC_H265_BACKEND=libde265` forces the fallback, to compare them on one machine.
    fn best() -> Result<Self> {
        let forced = std::env::var("ZDC_H265_BACKEND").unwrap_or_default();
        // every branch below says which decoder it picked and why. The two differ by nearly twice in speed (~83 fps of
        // 720p against ~50 on an RK3588 core), and a stale `ZDC_H265_BACKEND` in some unit file quietly costing that
        // much is exactly the kind of thing that gets blamed on the hardware for an afternoon.
        #[cfg(feature = "gstreamer")]
        let why = if forced == "libde265" {
            " (ZDC_H265_BACKEND asks for it; GStreamer's is faster where it is installed)".to_owned()
        } else {
            match crate::encoders::gstreamer::H265Decoder::new() {
                Ok(decoder) => {
                    log::info!("H.265 stream decoding with GStreamer {}", decoder.element());
                    return Ok(Backend::Gstreamer(decoder));
                }
                Err(error) => format!(" (no GStreamer H.265 decoder: {error:#})"),
            }
        };
        #[cfg(not(feature = "gstreamer"))]
        let why = String::new();
        let _ = &forced;
        let stream = h265::Stream::new().context("no H.265 decoder: neither a GStreamer one nor libde265")?;
        log::info!("H.265 stream decoding with libde265{why}");
        Ok(Backend::Libde265(stream))
    }

    /// One access unit in, a picture out with the `frame_id` of the unit it came from — which is an earlier unit's for
    /// a decoder that holds frames. `keyframe`: the unit is decodable on its own.
    fn decode(&mut self, access_unit: &[u8], keyframe: bool, size: Option<(u32, u32)>, frame_id: u64) -> Result<Option<(VideoImage, u64)>> {
        match self {
            #[cfg(feature = "gstreamer")]
            Backend::Gstreamer(decoder) => {
                // GStreamer is handed the size because nothing in the pipeline reports it back; before any parameter
                // set has said what it is there is also no keyframe to start from, so there is nothing to decode yet
                let Some(size) = size else { return Ok(None) };
                // a fresh pipeline cannot use a predicted frame, and pushing one anyway costs a full output timeout
                // waiting for a picture that will never come - seconds per frame, which is far worse than the second
                // of pictures a resync should cost
                if decoder.needs_keyframe() && !keyframe {
                    return Ok(None);
                }
                let Some((data, frame_id)) = decoder.decode(access_unit, size, frame_id)? else { return Ok(None) };
                VideoImage::i420(size.0, size.1, data).map(|picture| Some((picture, frame_id)))
            }
            // libde265 reports its own picture size, so it has no use for `size` (which only GStreamer needs told), and
            // it holds no frames: the picture it gives is the unit just pushed
            Backend::Libde265(stream) => {
                let _ = size;
                Ok(stream.push(access_unit, keyframe)?.map(|picture| (picture, frame_id)))
            }
        }
    }

    fn resync(&mut self) {
        match self {
            // its pipeline is kept: a keyframe clears the reference pictures by itself, and rebuilding on every lost
            // sample is ~35 ms of stall each on a decoder that was working
            #[cfg(feature = "gstreamer")]
            Backend::Gstreamer(decoder) => decoder.resync(),
            Backend::Libde265(stream) => stream.resync(),
        }
    }

    /// Pipelines the decoder has built (0 where there is no pipeline to build).
    fn pipelines_made(&self) -> u64 {
        match self {
            #[cfg(feature = "gstreamer")]
            Backend::Gstreamer(decoder) => decoder.pipelines_made(),
            Backend::Libde265(_) => 0,
        }
    }

    /// How many access units may be inside before a `decode` waits for its picture. libde265 holds none either way.
    fn hold(&mut self, frames: u64) {
        match self {
            #[cfg(feature = "gstreamer")]
            Backend::Gstreamer(decoder) => decoder.hold(frames),
            Backend::Libde265(_) => {}
        }
        let _ = frames;
    }
}

/// Keeps one H.265 decoder for a stream of `foxglove_msgs/CompressedVideo`, so its predicted frames decode.
///
/// The decoder is made on the first H.265 sample rather than up front: the gateway asks for one of these before any
/// sample has arrived, so the format is not known yet, and a stream that turns out to be H.264 should not have started
/// libde265 and its threads for nothing.
pub struct CompressedVideoStream {
    parse: Parse,
    h265: Option<Backend>,
    /// the newest parameter sets, to put back in front of a keyframe that arrives without them
    parameter_sets: Option<Vec<u8>>,
    /// the newest size any sequence parameter set gave: the units between keyframes carry none
    size: Option<(u32, u32)>,
    /// which sample the last picture came from, for the gateway to stamp it with ([`StreamDecoder::last_frame_id`])
    last_frame_id: Option<u64>,
    /// the headers of the samples given to the decoder, oldest first, with the number each was given: a decoder that
    /// holds a frame gives back an earlier sample's picture, and that picture's `stamp` and `frameId` — which the page
    /// reads to know when and where the picture was taken — are that sample's, not the one just pushed in
    headers: VecDeque<(u64, Header)>,
    /// how many samples the decoder may hold, to set on it once it is made ([`CompressedVideoStream::hold`])
    hold: Option<u64>,
}

/// Headers kept waiting for their picture. The decoder holds one frame, so two is the working depth; the rest is slack
/// for a unit that completes no picture, whose header is dropped when a later one's picture comes out.
const HEADERS_MAX: usize = 8;

impl CompressedVideoStream {
    /// A decoder for an encoding that reads its messages with `parse`.
    pub fn new(parse: Parse) -> Self {
        CompressedVideoStream { parse, h265: None, parameter_sets: None, size: None, last_frame_id: None, headers: VecDeque::new(), hold: None }
    }

    /// Decoder pipelines this stream has built, where its decoder builds one (GStreamer): **one** for a stream, for
    /// its life. A lost sample does not need another — a keyframe clears the reference pictures by itself — and one
    /// per lost sample cost ~35 ms of stall each on a decoder that was working.
    pub fn pipelines_made(&self) -> u64 {
        self.h265.as_ref().map_or(0, Backend::pipelines_made)
    }

    /// How many samples the decoder may hold before it waits for a picture, over what the decoder picks for itself.
    ///
    /// `0` is one decode and one encode in series per frame, which is what capped a 720p stream at ~22 fps on the
    /// robot — but also the only setting under which a sample's picture comes out of the same call that fed it, which
    /// is what the tests of what a single sample means are written against.
    pub fn hold(&mut self, frames: u64) {
        self.hold = Some(frames);
        if let Some(backend) = &mut self.h265 {
            backend.hold(frames);
        }
    }
}

impl StreamDecoder for CompressedVideoStream {
    fn decode(&mut self, sample: &EncodingSample<'_>, frame_id: u64) -> Result<Option<DecodedFrame>> {
        let message = (self.parse)(sample.payload)?;
        match Codec::from_format(&message.format)? {
            Codec::H265 => {}
            // openh264's decoder would do the same job, but nothing here needs it yet and an untested path is worse
            // than a clear refusal: H.264 keyframes still play through the per-sample decode.
            Codec::H264 => bail!("this stream is H.264, which has no stateful decoder here (its keyframes still play one by one)"),
        }
        let unit = compressed_video::to_access_unit(message.data, &message.format)?;
        let (data, keyframe) = compressed_video::decodable(&unit, &mut self.parameter_sets);
        if let Some(size) = unit.size {
            self.size = Some(size);
        }
        let backend = match &mut self.h265 {
            Some(backend) => backend,
            none => {
                let mut made = Backend::best()?;
                if let Some(frames) = self.hold {
                    made.hold(frames);
                }
                none.insert(made)
            }
        };
        self.headers.push_back((frame_id, message.header.clone()));
        while self.headers.len() > HEADERS_MAX {
            self.headers.pop_front();
        }
        let Some((picture, picture_id)) = backend.decode(&data, keyframe, self.size, frame_id)? else { return Ok(None) };
        self.last_frame_id = Some(picture_id);
        // the samples before this picture's completed no picture of their own
        while self.headers.front().is_some_and(|(id, _)| *id < picture_id) {
            self.headers.pop_front();
        }
        // the picture's header is the header of the sample it came from, which for a decoder holding a frame is not the
        // one just read (`every_picture_carries_the_header_of_its_own_sample`). The fallback is that sample's: it is
        // only reached by a decoder that held pictures back past `HEADERS_MAX`, and a stamp a frame or two out beats none
        let header = self.headers.front().filter(|(id, _)| *id == picture_id).map_or(&message.header, |(_, header)| header);
        Ok(Some(DecodedFrame::Video(header.add_to_video(picture))))
    }

    fn last_frame_id(&self) -> Option<u64> {
        self.last_frame_id
    }

    fn resync(&mut self) {
        if let Some(backend) = &mut self.h265 {
            backend.resync();
        }
        // the samples whose pictures the decoder has thrown away: nothing will come out for them now
        self.headers.clear();
        // the parameter sets are kept: they describe the stream, not the frames that were lost
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codecs::msgs::ros2_compressed_video;
    use crate::utils::compressed_video::{Codec, nal_units};
    use crate::utils::tests::fixture;

    /// `foxglove_msgs/CompressedVideo` as CDR, so the decoder is fed what the gateway would feed it. `sec` stamps its
    /// header, so each sample can be told apart by the header its picture carries.
    fn cdr_at(data: &[u8], format: &str, sec: i32) -> Vec<u8> {
        let mut body: Vec<u8> = Vec::with_capacity(data.len() + 64);
        let align = |body: &mut Vec<u8>| while body.len() % 4 != 0 { body.push(0) };
        let string = |body: &mut Vec<u8>, text: &str| {
            align(body);
            body.extend_from_slice(&(text.len() as u32 + 1).to_le_bytes());
            body.extend_from_slice(text.as_bytes());
            body.push(0);
        };
        body.extend_from_slice(&sec.to_le_bytes());
        body.extend_from_slice(&0u32.to_le_bytes());
        string(&mut body, "camera_color_optical_frame");
        align(&mut body);
        body.extend_from_slice(&(data.len() as u32).to_le_bytes());
        body.extend_from_slice(data);
        string(&mut body, format);
        [&[0x00, 0x01, 0x00, 0x00][..], &body].concat()
    }

    fn decode(decoder: &mut CompressedVideoStream, data: &[u8], format: &str) -> Result<Option<DecodedFrame>> {
        feed(decoder, data, format, 0, 1_700_000_000)
    }

    /// One sample in, numbered `frame_id` as the gateway numbers them, with `sec` as its header's stamp.
    fn feed(decoder: &mut CompressedVideoStream, data: &[u8], format: &str, frame_id: u64, sec: i32) -> Result<Option<DecodedFrame>> {
        let payload = cdr_at(data, format, sec);
        let encoding = zenoh_gateway::zenoh::bytes::Encoding::default();
        decoder.decode(&EncodingSample::new("camera/front", &payload, &encoding), frame_id)
    }

    /// The access units of the robot's capture: its parameter sets are not in band, so only the keyframe fixture has
    /// them, and the publisher is what puts them in front — which is what `decodable` stands in for.
    fn capture() -> (Vec<u8>, Vec<Vec<u8>>) {
        let predicted = nal_units(Codec::H265, &fixture("compressed/m20_front_2s.h265"))
            .into_iter()
            .filter(|(kind, _)| *kind == 1)
            .map(|(_, unit)| [&[0, 0, 0, 1][..], unit].concat())
            .collect();
        (fixture("compressed/m20_front_keyframe.h265"), predicted)
    }

    /// Both backends decode the robot's real stream, so the choice between them is speed, not capability. Run with
    /// `--nocapture` to see which each one used and how fast it was.
    #[test]
    fn every_backend_decodes_the_real_stream_at_the_same_size() {
        let (keyframe, predicted) = capture();
        let mut any = false;
        for forced in ["", "libde265"] {
            // SAFETY: single-threaded test, and the variable is only read when a backend is made
            unsafe { std::env::set_var("ZDC_H265_BACKEND", forced) };
            let Ok(mut backend) = Backend::best() else {
                eprintln!("no backend for ZDC_H265_BACKEND={forced:?}: skipping");
                continue;
            };
            let name = match &backend {
                #[cfg(feature = "gstreamer")]
                Backend::Gstreamer(decoder) => format!("gstreamer {}", decoder.element()),
                Backend::Libde265(_) => "libde265".to_owned(),
            };
            // every picture out of the call that fed it, so the count is exact and nothing overlaps the decode this
            // measures (what the pictures cost is the point here; what holding one buys is
            // `a_held_picture_comes_out_of_a_later_call`'s)
            backend.hold(0);
            let mut sets = None;
            let unit = compressed_video::to_access_unit(&keyframe, "h265").unwrap();
            let (data, is_key) = compressed_video::decodable(&unit, &mut sets);
            let size = unit.size.expect("the keyframe carries an SPS");
            let started = std::time::Instant::now();
            let (first, first_id) = backend.decode(&data, is_key, Some(size), 0).unwrap().expect("the keyframe decodes");
            assert_eq!((first.width(), first.height()), (1280, 720), "{name}");
            assert_eq!(first_id, 0, "{name}: the picture is the sample it was given");
            let mut pictures = 1;
            for (index, unit) in predicted.iter().enumerate() {
                let unit = compressed_video::to_access_unit(unit, "h265").unwrap();
                let (data, is_key) = compressed_video::decodable(&unit, &mut sets);
                let frame_id = index as u64 + 1;
                if let Some((_, picture_id)) = backend.decode(&data, is_key, Some(size), frame_id).unwrap() {
                    assert_eq!(picture_id, frame_id, "{name}: holding nothing, each picture is the sample just fed");
                    pictures += 1;
                }
            }
            let elapsed = started.elapsed().as_secs_f64();
            assert_eq!(pictures, 1 + predicted.len(), "{name} decoded only {pictures} of {}", 1 + predicted.len());
            eprintln!("{name}: {pictures} pictures of 1280x720 in {:.0} ms = {:.0} fps", elapsed * 1000.0, pictures as f64 / elapsed);
            any = true;
        }
        // SAFETY: as above
        unsafe { std::env::remove_var("ZDC_H265_BACKEND") };
        assert!(any, "no H.265 decoder at all on this machine");
    }

    #[test]
    fn decodes_the_predicted_frames_a_per_sample_decode_cannot() {
        if h265::Stream::new().is_err() {
            eprintln!("skipping: libde265 is not installed");
            return;
        }
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        // what one sample makes of itself, which is only visible while the decoder holds nothing back
        decoder.hold(0);
        let first = decode(&mut decoder, &keyframe, "h265").unwrap().expect("the keyframe decodes");
        let DecodedFrame::Video(image) = first else { panic!("a picture") };
        assert_eq!((image.width(), image.height()), (1280, 720));
        // its message's header rides along, as it does on the per-sample path
        assert_eq!(image.message_header().map(|header| header.frame_id.clone()), Some("camera_color_optical_frame".to_owned()));
        let mut pictures = 1;
        for unit in &predicted {
            if decode(&mut decoder, unit, "h265").unwrap().is_some() {
                pictures += 1;
            }
        }
        assert_eq!(pictures, 1 + predicted.len(), "every predicted frame decoded, not just the keyframe");
    }

    #[test]
    fn shows_nothing_until_a_keyframe_and_again_after_a_gap() {
        if h265::Stream::new().is_err() {
            return;
        }
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        decoder.hold(0);
        // before any keyframe there is nothing to predict from
        assert!(decode(&mut decoder, &predicted[0], "h265").unwrap().is_none());
        assert!(decode(&mut decoder, &keyframe, "h265").unwrap().is_some());
        assert!(decode(&mut decoder, &predicted[0], "h265").unwrap().is_some());
        // after a gap it waits for the next keyframe rather than showing a picture built on frames it never saw
        decoder.resync();
        assert!(decode(&mut decoder, &predicted[1], "h265").unwrap().is_none());
        assert!(decode(&mut decoder, &keyframe, "h265").unwrap().is_some());
    }

    /// The decoder holds a frame, so the picture a call hands over is an earlier sample's — and it has to come with
    /// **that** sample's header, since `stamp` and `frameId` are how the page knows when and where a picture was taken.
    /// Paired by position instead of by frame id, every picture of the stream would carry the next sample's stamp.
    #[test]
    fn every_picture_carries_the_header_of_its_own_sample() {
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        let mut units = vec![keyframe];
        units.extend(predicted.iter().take(9).cloned());
        let mut pictures = Vec::new();
        for (index, unit) in units.iter().enumerate() {
            let frame_id = index as u64;
            // each sample's header is stamped with its own number, so a picture says which sample it came from
            let Ok(decoded) = feed(&mut decoder, unit, "h265", frame_id, 1_000 + frame_id as i32) else {
                eprintln!("skipping: no H.265 decoder on this machine");
                return;
            };
            let Some(DecodedFrame::Video(picture)) = decoded else { continue };
            let header = picture.message_header().expect("the picture keeps its message's header");
            pictures.push((decoder.last_frame_id().expect("a picture names its sample"), header.sec, frame_id));
        }
        assert!(pictures.len() + 1 >= units.len(), "{} pictures of {} samples", pictures.len(), units.len());
        for (picture_id, sec, fed_id) in &pictures {
            assert_eq!(*sec, 1_000 + *picture_id as i32, "the picture of sample {picture_id} carries sample {}'s header (it came out of sample {fed_id}'s call)", sec - 1_000);
        }
    }

    /// Why the pairing above is not theoretical: with GStreamer the picture a call gives back is the sample before it,
    /// which is what lets a picture decode while the gateway encodes the one before it.
    #[cfg(feature = "gstreamer")]
    #[test]
    fn a_held_picture_comes_out_of_a_later_call() {
        if crate::encoders::gstreamer::H265Decoder::new().is_err() {
            eprintln!("skipping: no GStreamer H.265 decoder");
            return;
        }
        // SAFETY: the variable is read only when a backend is made, and this asks for the default one
        unsafe { std::env::remove_var("ZDC_H265_BACKEND") };
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        let mut units = vec![keyframe];
        units.extend(predicted.iter().take(4).cloned());
        let mut held = 0;
        for (index, unit) in units.iter().enumerate() {
            let frame_id = index as u64;
            if feed(&mut decoder, unit, "h265", frame_id, 1_000 + frame_id as i32).unwrap().is_some() && decoder.last_frame_id() != Some(frame_id) {
                held += 1;
            }
        }
        assert!(held > 0, "every picture came out of the call that fed it: the decoder is waiting for each one, which is the ~22 fps ceiling");
    }

    /// A lost sample must not cost a new decoder. Rebuilding the GStreamer pipeline on every resync stalls the loop
    /// ~35 ms each time, and a stream that loses a sample every few seconds pays it again and again — on a decoder
    /// that was working. A keyframe is all a gap needs: an IRAP picture clears the reference pictures by itself.
    #[cfg(feature = "gstreamer")]
    #[test]
    fn a_lost_sample_does_not_build_a_new_decoder() {
        if crate::encoders::gstreamer::H265Decoder::new().is_err() {
            eprintln!("skipping: no GStreamer H.265 decoder");
            return;
        }
        // SAFETY: single-threaded test, and the variable is read only when a backend is made
        unsafe { std::env::remove_var("ZDC_H265_BACKEND") };
        let (keyframe, predicted) = capture();
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        decoder.hold(0);
        // this stream's own count, not the process-wide one: other tests in this binary have decoders of their own
        let before = decoder.pipelines_made();
        assert!(feed(&mut decoder, &keyframe, "h265", 0, 1_000).unwrap().is_some(), "the keyframe decodes");
        for (index, unit) in predicted.iter().take(3).enumerate() {
            feed(&mut decoder, unit, "h265", index as u64 + 1, 1_001 + index as i32).unwrap();
        }
        let started = decoder.pipelines_made();
        assert_eq!((before, started), (0, 1), "one stream, one pipeline");
        // three samples lost, three times over: each is a resync
        for round in 0..3 {
            decoder.resync();
            // nothing until the keyframe, as before
            assert!(feed(&mut decoder, &predicted[4], "h265", 10 + round, 2_000).unwrap().is_none(), "a predicted frame after a gap shows nothing");
            assert!(feed(&mut decoder, &keyframe, "h265", 11 + round, 2_001).unwrap().is_some(), "the next keyframe makes it whole again");
        }
        assert_eq!(decoder.pipelines_made(), started, "three resyncs, still the one pipeline");
        // and it is still decoding predicted frames off that keyframe
        let pictures = predicted.iter().take(5).enumerate().filter(|(index, unit)| feed(&mut decoder, unit, "h265", 20 + *index as u64, 3_000).unwrap().is_some()).count();
        assert_eq!(pictures, 5, "the stream carries on after the resyncs");
    }

    #[test]
    fn says_why_it_cannot_take_h264() {
        let mut decoder = CompressedVideoStream::new(ros2_compressed_video::parse);
        let error = decode(&mut decoder, &fixture("compressed/pattern.h264"), "h264").unwrap_err().to_string();
        assert!(error.contains("no stateful decoder here"), "{error}");
    }
}
