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
const DE265_ERROR_WAITING_FOR_INPUT_DATA: c_int = 13;
/// the warnings that say nothing about the picture (threading, SEI, the warning queue itself); every other one means it
/// decoded damaged (libde265 conceals corrupt slices and missing references, warning instead of failing)
const DE265_HARMLESS_WARNINGS: [c_int; 5] = [1000, 1001, 1022, 1025, 1036];
const DE265_CHROMA_MONO: c_int = 0;
const DE265_CHROMA_420: c_int = 1;

/// The libde265 functions used, from the library they live in (kept loaded with them).
struct Api {
    new_decoder: unsafe extern "C" fn() -> Context_,
    free_decoder: unsafe extern "C" fn(Context_) -> c_int,
    push_data: unsafe extern "C" fn(Context_, *const c_void, c_int, i64, *mut c_void) -> c_int,
    flush_data: unsafe extern "C" fn(Context_) -> c_int,
    push_end_of_frame: unsafe extern "C" fn(Context_),
    get_warning: unsafe extern "C" fn(Context_) -> c_int,
    decode: unsafe extern "C" fn(Context_, *mut c_int) -> c_int,
    get_next_picture: unsafe extern "C" fn(Context_) -> Image,
    get_image_width: unsafe extern "C" fn(Image, c_int) -> c_int,
    get_image_height: unsafe extern "C" fn(Image, c_int) -> c_int,
    get_chroma_format: unsafe extern "C" fn(Image) -> c_int,
    get_bits_per_pixel: unsafe extern "C" fn(Image, c_int) -> c_int,
    get_image_plane: unsafe extern "C" fn(Image, c_int, *mut c_int) -> *const u8,
    get_error_text: unsafe extern "C" fn(c_int) -> *const c_char,
    is_ok: unsafe extern "C" fn(c_int) -> c_int,
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
                push_end_of_frame: symbol!("de265_push_end_of_frame"),
                get_warning: symbol!("de265_get_warning"),
                decode: symbol!("de265_decode"),
                get_next_picture: symbol!("de265_get_next_picture"),
                get_image_width: symbol!("de265_get_image_width"),
                get_image_height: symbol!("de265_get_image_height"),
                get_chroma_format: symbol!("de265_get_chroma_format"),
                get_bits_per_pixel: symbol!("de265_get_bits_per_pixel"),
                get_image_plane: symbol!("de265_get_image_plane"),
                get_error_text: symbol!("de265_get_error_text"),
                is_ok: symbol!("de265_isOK"),
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

/// A decoder for a stream of access units, each decoded with the state the ones before it left.
pub struct Stream {
    decoder: Decoder<'static>,
}

// SAFETY: the context is used by one thread at a time (`feed` takes `&mut self`), and libde265 keeps no thread-local state
unsafe impl Send for Stream {}

impl Stream {
    pub fn new() -> Result<Stream> {
        let api = api()?;
        // SAFETY: a fresh context, freed by the guard
        let decoder = Decoder { api, context: unsafe { (api.new_decoder)() } };
        anyhow::ensure!(!decoder.context.is_null(), "libde265 made no decoder");
        Ok(Stream { decoder })
    }

    /// Decodes the next access unit: the newest picture it finished, if any. Fails on a decoding error, and on a picture
    /// libde265 warns it decoded damaged (it would show garbage until the next keyframe).
    pub fn feed(&mut self, data: &[u8]) -> Result<Option<VideoImage>> {
        let (api, context) = (self.decoder.api, self.decoder.context);
        let length = c_int::try_from(data.len()).context("H.265 access unit over 2 GB")?;
        // SAFETY: libde265 copies the pushed bytes; `data` outlives the call. The access unit ends here, so its last NAL
        // decodes now rather than when the next sample's start code arrives.
        let pushed = unsafe { (api.push_data)(context, data.as_ptr().cast(), length, 0, std::ptr::null_mut()) };
        if pushed != DE265_OK {
            bail!("libde265: {}", api.error_text(pushed));
        }
        // SAFETY: as above
        unsafe { (api.push_end_of_frame)(context) };
        let mut newest = None;
        loop {
            let mut more: c_int = 0;
            // SAFETY: as above; `more` is a valid out pointer
            let error = unsafe { (api.decode)(context, &mut more) };
            // SAFETY: each picture is copied before any other libde265 call
            loop {
                let picture = unsafe { (api.get_next_picture)(context) };
                if picture.is_null() {
                    break;
                }
                newest = Some(copy_picture(api, picture)?);
            }
            if error == DE265_ERROR_WAITING_FOR_INPUT_DATA || more == 0 {
                break;
            }
            // SAFETY: de265_isOK takes any code
            if unsafe { (api.is_ok)(error) } == 0 {
                bail!("libde265: {}", api.error_text(error));
            }
        }
        let mut damage = None;
        loop {
            // SAFETY: as above
            let warning = unsafe { (api.get_warning)(context) };
            if warning == DE265_OK {
                break;
            }
            if !DE265_HARMLESS_WARNINGS.contains(&warning) {
                damage.get_or_insert(warning);
            }
        }
        if let Some(warning) = damage {
            bail!("libde265: {} (a corrupt or lost sample)", api.error_text(warning));
        }
        Ok(newest)
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
pub(crate) mod tests {
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

    #[test]
    fn says_why_a_bare_slice_has_no_picture() {
        if api().is_err() {
            return;
        }
        assert!(decode(&fixture("compressed/pattern_bare.h265")).is_err());
    }
}
