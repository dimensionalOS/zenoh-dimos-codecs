//! H.265 decoding with libde265, loaded at runtime (dlopen, so building needs nothing; Ubuntu's `libde265-0`, Homebrew's
//! `libde265`). There is no maintained pure-Rust HEVC decoder, and libde265's C API takes no struct layouts, so one
//! loader works with every 1.x release.

use super::compressed_video::i420_from_planes;
use anyhow::{Context, Result, bail};
use libloading::Library;
use std::ffi::{CStr, c_char, c_int, c_void};
use std::sync::OnceLock;
use zenoh_gateway::VideoImage;

type Context_ = *mut c_void;
type Image = *const c_void;

const DE265_OK: c_int = 0;
const DE265_CHROMA_MONO: c_int = 0;
const DE265_CHROMA_420: c_int = 1;

/// The libde265 functions used, from the library they live in (kept loaded with them).
struct Api {
    new_decoder: unsafe extern "C" fn() -> Context_,
    free_decoder: unsafe extern "C" fn(Context_) -> c_int,
    push_data: unsafe extern "C" fn(Context_, *const c_void, c_int, i64, *mut c_void) -> c_int,
    flush_data: unsafe extern "C" fn(Context_) -> c_int,
    decode: unsafe extern "C" fn(Context_, *mut c_int) -> c_int,
    get_next_picture: unsafe extern "C" fn(Context_) -> Image,
    get_image_width: unsafe extern "C" fn(Image, c_int) -> c_int,
    get_image_height: unsafe extern "C" fn(Image, c_int) -> c_int,
    get_chroma_format: unsafe extern "C" fn(Image) -> c_int,
    get_bits_per_pixel: unsafe extern "C" fn(Image, c_int) -> c_int,
    get_image_plane: unsafe extern "C" fn(Image, c_int, *mut c_int) -> *const u8,
    get_error_text: unsafe extern "C" fn(c_int) -> *const c_char,
    is_ok: unsafe extern "C" fn(c_int) -> c_int,
    /// tells the decoder the access unit just pushed is complete, so it decodes it now instead of waiting for the next
    /// one's first bytes (one frame of latency, every frame)
    push_end_of_frame: unsafe extern "C" fn(Context_),
    /// back to the state of a new decoder, keeping the context: what a stream does when it has lost samples
    reset: unsafe extern "C" fn(Context_),
    start_worker_threads: unsafe extern "C" fn(Context_, c_int) -> c_int,
    _library: Library,
}

impl Api {
    fn load() -> Result<Api> {
        let names = ["libde265.so.0", "libde265.0.dylib", "libde265.dylib", "/opt/homebrew/lib/libde265.0.dylib", "/usr/local/lib/libde265.0.dylib"];
        let mut errors = Vec::new();
        // SAFETY: libde265 has no load-time side effects beyond its own initializers
        let library = names.iter().find_map(|name| unsafe { Library::new(*name) }.map_err(|error| errors.push(error.to_string())).ok());
        let Some(library) = library else {
            bail!("H.265 needs libde265 (apt install libde265-0, brew install libde265): {}", errors.join("; "));
        };
        // SAFETY: each symbol is looked up by its C name and given its signature from de265.h
        unsafe {
            macro_rules! symbol {
                ($name:literal) => {
                    *library.get(concat!($name, "\0").as_bytes()).with_context(|| concat!("libde265 has no ", $name))?
                };
            }
            Ok(Api {
                new_decoder: symbol!("de265_new_decoder"),
                free_decoder: symbol!("de265_free_decoder"),
                push_data: symbol!("de265_push_data"),
                flush_data: symbol!("de265_flush_data"),
                decode: symbol!("de265_decode"),
                get_next_picture: symbol!("de265_get_next_picture"),
                get_image_width: symbol!("de265_get_image_width"),
                get_image_height: symbol!("de265_get_image_height"),
                get_chroma_format: symbol!("de265_get_chroma_format"),
                get_bits_per_pixel: symbol!("de265_get_bits_per_pixel"),
                get_image_plane: symbol!("de265_get_image_plane"),
                get_error_text: symbol!("de265_get_error_text"),
                is_ok: symbol!("de265_isOK"),
                push_end_of_frame: symbol!("de265_push_end_of_frame"),
                reset: symbol!("de265_reset"),
                start_worker_threads: symbol!("de265_start_worker_threads"),
                _library: library,
            })
        }
    }

    fn error_text(&self, error: c_int) -> String {
        // SAFETY: de265_get_error_text returns a static NUL-terminated string for any code
        unsafe { CStr::from_ptr((self.get_error_text)(error)) }.to_string_lossy().into_owned()
    }
}

fn api() -> Result<&'static Api> {
    static API: OnceLock<Result<Api, String>> = OnceLock::new();
    API.get_or_init(|| Api::load().map_err(|error| format!("{error:#}"))).as_ref().map_err(|error| anyhow::anyhow!("{error}"))
}

/// A decoder context, freed on drop.
struct Decoder<'a> {
    api: &'a Api,
    context: Context_,
}

impl Drop for Decoder<'_> {
    fn drop(&mut self) {
        // SAFETY: the context came from de265_new_decoder and is freed once
        unsafe { (self.api.free_decoder)(self.context) };
    }
}

/// The first picture in an Annex-B access unit (with its parameter sets), as I420.
pub fn decode(data: &[u8]) -> Result<VideoImage> {
    let api = api()?;
    // SAFETY: a fresh context, used by this thread only, freed by the guard
    let decoder = Decoder { api, context: unsafe { (api.new_decoder)() } };
    anyhow::ensure!(!decoder.context.is_null(), "libde265 made no decoder");
    let length = c_int::try_from(data.len()).context("H.265 access unit over 2 GB")?;
    // SAFETY: libde265 copies the pushed bytes; `data` outlives the call
    let pushed = unsafe { (api.push_data)(decoder.context, data.as_ptr().cast(), length, 0, std::ptr::null_mut()) };
    if pushed != DE265_OK {
        bail!("libde265: {}", api.error_text(pushed));
    }
    // SAFETY: as above
    unsafe { (api.flush_data)(decoder.context) };
    loop {
        let mut more: c_int = 0;
        // SAFETY: as above; `more` is a valid out pointer
        let error = unsafe { (api.decode)(decoder.context, &mut more) };
        // SAFETY: as above; the picture is read before any other libde265 call
        let picture = unsafe { (api.get_next_picture)(decoder.context) };
        if !picture.is_null() {
            return copy_picture(api, picture);
        }
        // SAFETY: de265_isOK takes any code
        if unsafe { (api.is_ok)(error) } == 0 || more == 0 {
            bail!("libde265 decoded no picture ({})", if error == DE265_OK { "the access unit has none".to_owned() } else { api.error_text(error) });
        }
    }
}

/// The picture as I420, copied out of libde265 (it is only valid until the next libde265 call).
fn copy_picture(api: &Api, picture: Image) -> Result<VideoImage> {
    // SAFETY: `picture` is the decoder's current output, read before any other call
    unsafe {
        let chroma = (api.get_chroma_format)(picture);
        let (width, height) = ((api.get_image_width)(picture, 0) as usize, (api.get_image_height)(picture, 0) as usize);
        for channel in 0..if chroma == DE265_CHROMA_MONO { 1 } else { 3 } {
            let bits = (api.get_bits_per_pixel)(picture, channel);
            anyhow::ensure!(bits == 8, "H.265 picture is {bits}-bit (8-bit only)");
        }
        let plane = |channel: c_int, rows: usize| {
            let mut stride: c_int = 0;
            let pointer = (api.get_image_plane)(picture, channel, &mut stride);
            (std::slice::from_raw_parts(pointer, stride as usize * rows), stride as usize)
        };
        match chroma {
            DE265_CHROMA_420 => i420_from_planes(width, height, [plane(0, height), plane(1, height.div_ceil(2)), plane(2, height.div_ceil(2))]),
            DE265_CHROMA_MONO => {
                let grey = vec![128; width.div_ceil(2) * height.div_ceil(2)];
                i420_from_planes(width, height, [plane(0, height), (&grey, width.div_ceil(2)), (&grey, width.div_ceil(2))])
            }
            other => bail!("H.265 picture has chroma format {other} (4:2:0 or monochrome only)"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::tests::fixture;

    /// The pattern's decode, or `None` (and a note) where libde265 isn't installed.
    pub(crate) fn decode_pattern() -> Option<VideoImage> {
        if let Err(error) = api() {
            eprintln!("skipping: {error:#}");
            return None;
        }
        Some(decode(&fixture("compressed/pattern.h265")).unwrap())
    }

    #[test]
    fn decodes_a_keyframe() {
        let Some(image) = decode_pattern() else { return };
        assert_eq!((image.width(), image.height()), (320, 240));
        crate::utils::compressed_video::tests::assert_like_pattern(&image, "h265 keyframe");
    }

    /// The parameter sets an M20 keyframe carries, Annex-B, as the relay prepends them.
    fn m20_parameter_sets() -> Vec<u8> {
        let au = fixture("compressed/m20_front_keyframe.h265");
        let mut sets = Vec::new();
        for (kind, unit) in crate::utils::compressed_video::nal_units(crate::utils::compressed_video::Codec::H265, &au) {
            if (32..=34).contains(&kind) {
                sets.extend_from_slice(&[0, 0, 0, 1]);
                sets.extend_from_slice(unit);
            }
        }
        sets
    }

    /// The point of [`Stream`]: every frame of a real stream decodes, not just the keyframes.
    ///
    /// The capture is the camera's own output, one slice per picture and no parameter sets in band, so the keyframes
    /// get the relay's in front of them exactly as the gateway will. `decode` would give a picture for the two
    /// keyframes in this and nothing for the other thirty-two.
    #[test]
    fn a_stream_decodes_predicted_frames_too() {
        if api().is_err() {
            eprintln!("skipping: libde265 is not installed");
            return;
        }
        let sets = m20_parameter_sets();
        let capture = fixture("compressed/m20_front_2s.h265");
        let units = crate::utils::compressed_video::nal_units(crate::utils::compressed_video::Codec::H265, &capture);
        let mut stream = Stream::new().unwrap();
        let (mut pictures, mut predicted) = (0, 0);
        let started = std::time::Instant::now();
        for (kind, unit) in &units {
            let keyframe = (16..=21).contains(kind);
            let mut access_unit = if keyframe { sets.clone() } else { Vec::new() };
            access_unit.extend_from_slice(&[0, 0, 0, 1]);
            access_unit.extend_from_slice(unit);
            let picture = stream.push(&access_unit, keyframe).unwrap().expect("every unit of this capture completes a picture");
            assert_eq!((picture.width(), picture.height()), (1280, 720));
            pictures += 1;
            predicted += usize::from(!keyframe);
        }
        assert_eq!(pictures, units.len(), "every unit gave a picture");
        assert!(predicted > 20, "and {predicted} of them were predicted frames");
        eprintln!("{pictures} pictures of 1280x720 in {:.0} ms ({:.0} fps, {} worker threads)", started.elapsed().as_secs_f64() * 1000.0, pictures as f64 / started.elapsed().as_secs_f64(), worker_threads());
    }

    /// A predicted frame before any keyframe is skipped, not guessed at, and a gap starts that over.
    #[test]
    fn a_stream_waits_for_a_keyframe_before_and_after_a_gap() {
        if api().is_err() {
            return;
        }
        let mut stream = Stream::new().unwrap();
        // a bare TRAIL_R with nothing before it: nothing to predict from
        assert!(stream.push(&[0, 0, 0, 1, 0x02, 0x01, 0xaa], false).unwrap().is_none(), "no picture before a keyframe");
        let sets = m20_parameter_sets();
        let capture = fixture("compressed/m20_front_2s.h265");
        let units = crate::utils::compressed_video::nal_units(crate::utils::compressed_video::Codec::H265, &capture);
        let (kind, first) = units.iter().find(|(kind, _)| (16..=21).contains(kind)).expect("a keyframe");
        let mut keyframe = sets.clone();
        keyframe.extend_from_slice(&[0, 0, 0, 1]);
        keyframe.extend_from_slice(first);
        assert!(stream.push(&keyframe, (16..=21).contains(kind)).unwrap().is_some(), "the keyframe starts it");
        // after a gap the decoder is back to waiting, whatever is fed next
        stream.resync();
        let (_, predicted) = units.iter().find(|(kind, _)| *kind == 1).expect("a predicted frame");
        let mut unit = vec![0, 0, 0, 1];
        unit.extend_from_slice(predicted);
        assert!(stream.push(&unit, false).unwrap().is_none(), "a predicted frame after a gap is skipped");
        assert!(stream.push(&keyframe, true).unwrap().is_some(), "and the next keyframe starts it again");
    }

    #[test]
    fn says_why_a_bare_slice_has_no_picture() {
        if api().is_err() {
            return;
        }
        assert!(decode(&fixture("compressed/pattern_bare.h265")).is_err());
    }
}

/// How many worker threads a [`Stream`] decoder gets, unless the settings say otherwise. HEVC at 720p30 is more
/// than one core can hold, and a context's workers are internal to it, so they do not disturb the order access units are
/// fed in. Capped low because every camera has its own decoder.
/// Measured on one 720p capture from the robot (13th-gen i7, release): 1 thread 99 fps, 2 threads 151, 4 threads 192.
/// Two is the knee — it buys half again as much for one extra thread, while four would give two cameras eight threads
/// and leave the encoders nothing. Where this is not enough the answer is a hardware decoder, not more threads.
const DEFAULT_WORKER_THREADS: usize = 2;

/// Worker threads per decoder: [`Settings::h265_threads`](crate::settings::Settings::h265_threads), never more than
/// this machine has.
fn worker_threads() -> usize {
    let available = std::thread::available_parallelism().map_or(1, |count| count.get());
    crate::settings::settings().h265_threads.clamp(1, available.max(1))
}

/// A decoder that keeps its state between access units, so **predicted frames decode**.
///
/// [`decode`] makes a fresh context per call, which is right when each sample is sent on its own but means only a
/// keyframe ever produces a picture — on a camera sending one keyframe a second, 29 frames in 30 are lost. A `Stream`
/// is fed every access unit of one stream, in order, and gives back each picture as it completes.
///
/// It must be fed **in order and without gaps**: a predicted frame refers to pictures before it. After a gap, call
/// [`resync`](Self::resync); the decoder then skips everything until the next keyframe rather than showing a picture
/// built on frames it never saw. [`push`](Self::push) returns `Ok(None)` while it is resyncing.
pub struct Stream {
    api: &'static Api,
    context: Context_,
    /// skipping until the next keyframe: nothing decodable has been fed yet, or samples were lost
    resyncing: bool,
}

// SAFETY: a libde265 context is not thread safe, and `Stream` owns its context and lends it to nobody, so the context is
// only ever touched through `&mut self`. The gateway hands a `Stream` from one blocking task to the next, never to two
// at once, which is what `Send` (and not `Sync`) says.
unsafe impl Send for Stream {}

impl Drop for Stream {
    fn drop(&mut self) {
        // SAFETY: the context came from de265_new_decoder and is freed once, here
        unsafe { (self.api.free_decoder)(self.context) };
    }
}

impl Stream {
    /// A decoder waiting for its first keyframe.
    pub fn new() -> Result<Self> {
        let api = api()?;
        // SAFETY: a fresh context, owned by this value and freed on drop
        let context = unsafe { (api.new_decoder)() };
        anyhow::ensure!(!context.is_null(), "libde265 made no decoder");
        let stream = Stream { api, context, resyncing: true };
        let threads = worker_threads();
        // SAFETY: the context is this value's own and has decoded nothing yet
        let started = unsafe { (api.start_worker_threads)(stream.context, threads as c_int) };
        if started != DE265_OK {
            // not fatal: one thread decodes the same pictures, only slower
            log::warn!("libde265 would not start {threads} worker threads ({}): decoding on this thread", api.error_text(started));
        }
        Ok(stream)
    }

    /// Start over at the next keyframe, because samples before the next one were lost. Cheap: the context is kept.
    pub fn resync(&mut self) {
        // SAFETY: the context is this value's own; de265_reset leaves it as new
        unsafe { (self.api.reset)(self.context) };
        self.resyncing = true;
    }

    /// Feeds one Annex-B access unit, which follows every unit fed before it, and returns the picture it completed.
    ///
    /// `keyframe`: the unit is decodable on its own (an IRAP picture with its parameter sets). While resyncing,
    /// anything else is dropped unread and this gives `Ok(None)`.
    pub fn push(&mut self, data: &[u8], keyframe: bool) -> Result<Option<VideoImage>> {
        if self.resyncing {
            if !keyframe {
                return Ok(None);
            }
            self.resyncing = false;
        }
        let length = c_int::try_from(data.len()).context("H.265 access unit over 2 GB")?;
        // SAFETY: libde265 copies the pushed bytes; `data` outlives the call; the context is this value's own
        let pushed = unsafe { (self.api.push_data)(self.context, data.as_ptr().cast(), length, 0, std::ptr::null_mut()) };
        if pushed != DE265_OK {
            self.resync();
            bail!("libde265: {}", self.api.error_text(pushed));
        }
        // this unit is whole: decode it now rather than when the next one's first bytes arrive
        // SAFETY: as above
        unsafe { (self.api.push_end_of_frame)(self.context) };
        loop {
            let mut more: c_int = 0;
            // SAFETY: as above; `more` is a valid out pointer
            let error = unsafe { (self.api.decode)(self.context, &mut more) };
            // SAFETY: as above; the picture is copied out before any other libde265 call
            let picture = unsafe { (self.api.get_next_picture)(self.context) };
            if !picture.is_null() {
                return copy_picture(self.api, picture).map(Some);
            }
            // SAFETY: de265_isOK takes any code
            if unsafe { (self.api.is_ok)(error) } == 0 {
                // the stream is not what the decoder expected: everything after this may be built on it
                self.resync();
                bail!("libde265: {}", self.api.error_text(error));
            }
            if more == 0 {
                // nothing more to decode in what was pushed, and no picture: a unit that completes none (parameter sets
                // alone), or one the decoder is still holding
                return Ok(None);
            }
        }
    }
}
