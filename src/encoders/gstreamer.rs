//! H.264 through a GStreamer hardware encoder, found at runtime: `nvv4l2h264enc` (Jetson), `nvh264enc` (NVENC),
//! `vah264enc` / `vaapih264enc` (VAAPI). GStreamer is loaded with `dlopen`, so building needs no GStreamer and a
//! machine without it just falls back to software. One pipeline per picture size: `appsrc ! [upload] ! encoder !
//! h264parse ! appsink`, constant bitrate, no B-frames, bitrate changes in place, keyframes on request.

use anyhow::{Context, Result, anyhow, bail, ensure};
use libloading::Library;
use std::borrow::Cow;
use std::collections::VecDeque;
use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::ptr::null_mut;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};
use zenoh_gateway::{DecodedFrame, EncodedVideo, VideoEncoder, VideoFormat, VideoTarget};

type Pointer = *mut c_void;

/// The GStreamer C functions this backend calls (all ABI-stable since 1.10).
struct Api {
    init_check: unsafe extern "C" fn(*mut c_int, *mut c_void, *mut *mut GError) -> c_int,
    parse_launch: unsafe extern "C" fn(*const c_char, *mut *mut GError) -> Pointer,
    bin_get_by_name: unsafe extern "C" fn(Pointer, *const c_char) -> Pointer,
    element_factory_find: unsafe extern "C" fn(*const c_char) -> Pointer,
    element_set_state: unsafe extern "C" fn(Pointer, c_int) -> c_int,
    element_send_event: unsafe extern "C" fn(Pointer, Pointer) -> c_int,
    element_get_bus: unsafe extern "C" fn(Pointer) -> Pointer,
    bus_pop_filtered: unsafe extern "C" fn(Pointer, c_int) -> Pointer,
    message_parse_error: unsafe extern "C" fn(Pointer, *mut *mut GError, *mut *mut c_char),
    mini_object_unref: unsafe extern "C" fn(Pointer),
    object_unref: unsafe extern "C" fn(Pointer),
    util_set_object_arg: unsafe extern "C" fn(Pointer, *const c_char, *const c_char),
    buffer_new_allocate: unsafe extern "C" fn(Pointer, usize, Pointer) -> Pointer,
    buffer_fill: unsafe extern "C" fn(Pointer, usize, *const c_void, usize) -> usize,
    buffer_get_size: unsafe extern "C" fn(Pointer) -> usize,
    buffer_extract: unsafe extern "C" fn(Pointer, usize, *mut c_void, usize) -> usize,
    sample_get_buffer: unsafe extern "C" fn(Pointer) -> Pointer,
    app_src_push_buffer: unsafe extern "C" fn(Pointer, Pointer) -> c_int,
    app_sink_try_pull_sample: unsafe extern "C" fn(Pointer, u64) -> Pointer,
    force_key_unit: unsafe extern "C" fn(u64, u64, u64, c_int, u32) -> Pointer,
    class_find_property: unsafe extern "C" fn(Pointer, *const c_char) -> Pointer,
    error_free: unsafe extern "C" fn(*mut GError),
    free: unsafe extern "C" fn(Pointer),
    _libraries: Vec<Library>,
}

#[repr(C)]
struct GError {
    domain: u32,
    code: c_int,
    message: *const c_char,
}

const STATE_NULL: c_int = 1;
const STATE_PLAYING: c_int = 4;
const MESSAGE_ERROR: c_int = 1 << 1;
/// Every message type but errors (GST_MESSAGE_ANY without GST_MESSAGE_ERROR).
const MESSAGE_ANY_BUT_ERROR: c_int = !MESSAGE_ERROR;
const CLOCK_TIME_NONE: u64 = u64::MAX;

/// Opens the first of `names` that loads.
fn open(names: &[&str]) -> Result<Library> {
    let mut errors = Vec::new();
    for name in names {
        // SAFETY: GStreamer's libraries have no load-time side effects beyond their own initializers
        match unsafe { Library::new(*name) } {
            Ok(library) => return Ok(library),
            Err(error) => errors.push(error.to_string()),
        }
    }
    bail!("{}", errors.join("; "))
}

impl Api {
    fn load() -> Result<Api> {
        let names = |base: &str| [format!("lib{base}-1.0.so.0"), format!("lib{base}-1.0.0.dylib"), format!("lib{base}-1.0.dylib")];
        let load = |base: &str| open(&names(base).iter().map(String::as_str).collect::<Vec<_>>());
        let (core, app, video) = (load("gstreamer")?, load("gstapp")?, load("gstvideo")?);
        let gobject = open(&["libgobject-2.0.so.0", "libgobject-2.0.0.dylib"])?;
        let glib = open(&["libglib-2.0.so.0", "libglib-2.0.0.dylib"])?;
        // SAFETY: each symbol is looked up by its C name and given its documented C signature
        unsafe {
            macro_rules! symbol {
                ($library:expr, $name:literal) => {
                    *$library.get(concat!($name, "\0").as_bytes()).with_context(|| concat!("GStreamer has no ", $name))?
                };
            }
            Ok(Api {
                init_check: symbol!(core, "gst_init_check"),
                parse_launch: symbol!(core, "gst_parse_launch"),
                bin_get_by_name: symbol!(core, "gst_bin_get_by_name"),
                element_factory_find: symbol!(core, "gst_element_factory_find"),
                element_set_state: symbol!(core, "gst_element_set_state"),
                element_send_event: symbol!(core, "gst_element_send_event"),
                element_get_bus: symbol!(core, "gst_element_get_bus"),
                bus_pop_filtered: symbol!(core, "gst_bus_pop_filtered"),
                message_parse_error: symbol!(core, "gst_message_parse_error"),
                mini_object_unref: symbol!(core, "gst_mini_object_unref"),
                object_unref: symbol!(core, "gst_object_unref"),
                util_set_object_arg: symbol!(core, "gst_util_set_object_arg"),
                buffer_new_allocate: symbol!(core, "gst_buffer_new_allocate"),
                buffer_fill: symbol!(core, "gst_buffer_fill"),
                buffer_get_size: symbol!(core, "gst_buffer_get_size"),
                buffer_extract: symbol!(core, "gst_buffer_extract"),
                sample_get_buffer: symbol!(core, "gst_sample_get_buffer"),
                app_src_push_buffer: symbol!(app, "gst_app_src_push_buffer"),
                app_sink_try_pull_sample: symbol!(app, "gst_app_sink_try_pull_sample"),
                force_key_unit: symbol!(video, "gst_video_event_new_downstream_force_key_unit"),
                class_find_property: symbol!(gobject, "g_object_class_find_property"),
                error_free: symbol!(glib, "g_error_free"),
                free: symbol!(glib, "g_free"),
                _libraries: vec![core, app, video, gobject, glib],
            })
        }
    }

    /// A GError's message, freeing it.
    unsafe fn take_error(&self, error: *mut GError) -> String {
        if error.is_null() {
            return "unknown error".into();
        }
        // SAFETY: a GError GStreamer handed us, freed once
        unsafe {
            let message = CStr::from_ptr((*error).message).to_string_lossy().into_owned();
            (self.error_free)(error);
            message
        }
    }
}

/// GStreamer, loaded and initialized once per process.
fn api() -> Result<&'static Api> {
    static API: OnceLock<std::result::Result<Api, String>> = OnceLock::new();
    let api = API.get_or_init(|| {
        let api = Api::load().map_err(|error| format!("{error:#}"))?;
        let mut error = null_mut();
        // SAFETY: gst_init_check with no arguments
        if unsafe { (api.init_check)(null_mut(), null_mut(), &mut error) } == 0 {
            // SAFETY: the error init_check set
            return Err(format!("gst_init: {}", unsafe { api.take_error(error) }));
        }
        Ok(api)
    });
    api.as_ref().map_err(|error| anyhow!("GStreamer: {error}"))
}

/// How to drive one encoder element.
struct Element {
    name: &'static str,
    /// what turns system-memory I420 into what the encoder takes
    upload: &'static str,
    /// its `bitrate` property counts kbit/s (else bit/s)
    kbps: bool,
    /// low-latency constant-bitrate settings (ones the installed version lacks are skipped)
    settings: &'static [(&'static str, &'static str)],
    /// the properties that set the keyframe interval, in frames
    keyframe_interval: &'static [&'static str],
    /// the least width and height it encodes (smaller pictures are scaled up to it, keeping their shape)
    min_size: (u32, u32),
    /// the H.264 profile to hold the encoder to, as a caps filter behind it (`""`: whatever it negotiates).
    ///
    /// zenoh-gateway offers its H.264 tracks as `profile-level-id=42e01f`, constrained baseline
    /// ([`H264_SDP_PROFILE`]), so an encoder that negotiates something richer is sending a browser a stream it was
    /// not offered — which shows up as a black picture or a decoder error, not as a clean failure. The hardware
    /// elements are already configured for a baseline-ish profile by their own settings and are not all able to
    /// honour a caps filter, so this is set only where it is needed and testable.
    profile: &'static str,
}

/// The H.264 profile zenoh-gateway advertises in SDP (`profile-level-id=42e01f`), which an encoder behind it must
/// actually produce. Kept here beside the encoders so the two cannot drift apart silently, as they had.
pub const H264_SDP_PROFILE: &str = "constrained-baseline";

impl Element {
    /// What goes between this encoder and the appsink: the byte-stream access units zenoh-gateway wants, held to this
    /// encoder's [`profile`](Self::profile).
    ///
    /// `h264parse config-interval=-1` repeats the parameter sets before every keyframe, which a viewer joining
    /// mid-stream needs. It lives in `gstreamer1.0-plugins-bad`, which is not everywhere — and without it the pipeline
    /// would not build at all, so the software encoder was unavailable exactly where there is no hardware one to fall
    /// back from. Where the parser is missing, the encoder is asked for byte-stream directly instead: x264enc repeats
    /// SPS and PPS before each keyframe itself when its output is byte-stream (checked: `9,7,8,5,5,5` at every
    /// keyframe), and `nvv4l2h264enc` has `insert-sps-pps=true` among its settings for the same reason.
    fn output_chain(&self, api: &Api) -> String {
        self.output_chain_with(has_element(api, "h264parse"))
    }

    /// [`output_chain`](Self::output_chain) for a machine that has `h264parse` or not, so both shapes can be checked
    /// without installing GStreamer's parsers. They are checked because they differed by a comma and the wrong one
    /// could not be parsed at all: the profile carries a **leading** comma, which is right whether a field follows it
    /// or the caps end there. It used to carry a trailing one, correct only in the branch without the parser — which is
    /// the branch a machine without `gstreamer1.0-plugins-bad` takes, so every test passed while the robot, which has
    /// the parser, could not build the pipeline and silently encoded with openh264 instead.
    fn output_chain_with(&self, parser: bool) -> String {
        let profile = if self.profile.is_empty() { String::new() } else { format!(",profile={}", self.profile) };
        if parser {
            let filter = if profile.is_empty() { String::new() } else { format!(" video/x-h264{profile} !") };
            format!("{filter} h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au !")
        } else {
            format!(" video/x-h264{profile},stream-format=byte-stream,alignment=au !")
        }
    }
}

/// Whether GStreamer has the element called `name`.
fn has_element(api: &Api, name: &str) -> bool {
    // SAFETY: the factory reference is released at once
    unsafe {
        let factory = (api.element_factory_find)(c_string(name).as_ptr());
        let found = !factory.is_null();
        if found {
            (api.object_unref)(factory);
        }
        found
    }
}

/// Hardware encoders, in the order they are tried.
static HARDWARE: [Element; 4] = [
    Element {
        name: "nvv4l2h264enc",
        upload: "nvvidconv ! video/x-raw(memory:NVMM),format=I420 !",
        kbps: false,
        settings: &[("control-rate", "1"), ("insert-sps-pps", "true"), ("insert-vui", "true"), ("maxperf-enable", "true"), ("preset-level", "1"), ("profile", "0"), ("poc-type", "2")],
        keyframe_interval: &["iframeinterval", "idrinterval"],
        // under 160 wide or 64 high every frame failed (BlockSide error 0x4) on the R1's Orin, L4T 36
        min_size: (160, 64),
        // its own `profile: 0` setting above is baseline
        profile: "",
    },
    Element { profile: "", name: "nvh264enc", upload: "", kbps: true, settings: &[("rc-mode", "cbr"), ("preset", "low-latency-hp"), ("zerolatency", "true"), ("bframes", "0")], keyframe_interval: &["gop-size"], min_size: (0, 0) },
    Element { profile: "", name: "vah264enc", upload: "videoconvert ! video/x-raw,format=NV12 !", kbps: true, settings: &[("rate-control", "cbr"), ("b-frames", "0"), ("target-usage", "7")], keyframe_interval: &["key-int-max"], min_size: (0, 0) },
    Element { profile: "", name: "vaapih264enc", upload: "videoconvert ! video/x-raw,format=NV12 !", kbps: true, settings: &[("rate-control", "cbr"), ("max-bframes", "0")], keyframe_interval: &["keyframe-period"], min_size: (0, 0) },
];
/// Software, for trying the pipeline on a machine without a hardware encoder.
/// Software, for trying the pipeline on a machine without a hardware encoder — and the encoder of choice where there is
/// no usable hardware one at all: on an RK3588 (no V4L2 encoder nodes, no VAAPI) x264 at `ultrafast` does ~54 fps of
/// 720p per A76 core against openh264's ~27.
///
/// Held to constrained baseline: x264 negotiates high profile by default, which is not what the gateway offered.
static X264: Element = Element { name: "x264enc", upload: "", kbps: true, settings: &[("tune", "zerolatency"), ("speed-preset", "ultrafast"), ("bframes", "0")], keyframe_interval: &["key-int-max"], min_size: (0, 0), profile: H264_SDP_PROFILE };
/// The encoder whose pipeline takes JPEG (the Jetson's: nvjpegdec and nvvidconv in front of it).
pub(crate) const JPEG_ENCODER: &str = "nvv4l2h264enc";
/// Frames inside the encoder before `encode` waits for one to come out: hardware encoders take a few frame times
/// (nvv4l2h264enc ~40 ms on a loaded Orin), so waiting for each frame's own output capped a stream at ~25 fps.
///
/// It is also latency: a frame held is a frame not sent, and three of them at 30 Hz is up to 100 ms, which on a robot
/// whose encode costs 1.6 ms buys nothing. The gateway drains what is inside between inputs, so the cost is usually
/// less — `ZDC_GST_IN_FLIGHT` is there to measure what it actually is on a machine, against 1 and 0.
fn max_in_flight() -> u64 {
    static IN_FLIGHT: OnceLock<u64> = OnceLock::new();
    *IN_FLIGHT.get_or_init(|| std::env::var("ZDC_GST_IN_FLIGHT").ok().and_then(|value| value.parse().ok()).unwrap_or(3))
}
/// The same for a pipeline that takes JPEG: its decoder, converter and encoder each hold frames (on the R1, ~40 ms from
/// JPEG to H.264 idle, longer under load), and waiting for each output at 3 inside cost ~35 ms a frame, which zenoh-gateway's
/// CPU governor took for encoding cost and shrank the picture for. `ZDC_GST_JPEG_IN_FLIGHT` overrides it.
fn jpeg_in_flight() -> u64 {
    static IN_FLIGHT: OnceLock<u64> = OnceLock::new();
    *IN_FLIGHT.get_or_init(|| std::env::var("ZDC_GST_JPEG_IN_FLIGHT").ok().and_then(|value| value.parse().ok()).unwrap_or(6))
}
/// Access units inside the decoder before `decode` waits for a picture to come out.
///
/// One. Waiting for each unit's own picture puts the whole decode in the gateway's critical path, and the loop then runs
/// at decode + encode a frame: 26 ms + ~12 ms on an RK3588 A76 core capped a 720p stream at ~22 fps while the CPU sat
/// near idle — latency-bound, not CPU-bound. With one unit of slack a picture decodes while the gateway encodes the one
/// before it, so the two stages overlap and the slower one sets the rate.
///
/// The picture that comes out is the one pushed before this, so it is one sample interval old — but it also finished
/// decoding during that interval, so what this costs is `interval - decode`, not a whole interval: 33 - 26 = 7 ms at
/// 30 Hz on that core, for 22 -> 30 fps. Where the decode is slower than the interval it costs nothing at all. That is
/// also why it is one and not three: the gateway sends one picture per `decode` call, so deeper slack buys no more
/// overlap and does add a whole interval each. `ZDC_H265_FRAMES_INSIDE=0` waits for every picture, as before — the
/// serial behaviour, to measure against.
const FRAMES_INSIDE: u64 = 1;

fn frames_inside() -> u64 {
    std::env::var("ZDC_H265_FRAMES_INSIDE").ok().and_then(|value| value.parse().ok()).unwrap_or(FRAMES_INSIDE)
}
/// How long that wait may last (the first frame's includes the pipeline's start); under zenoh-gateway's default
/// `encode_timeout` (5 s), so the encode fails here instead of being abandoned on its thread.
const OUTPUT_TIMEOUT: Duration = Duration::from_secs(3);
/// A frame rate this many times off the pipeline's rebuilds it: zenoh-gateway's AV1 encoder's `RECONFIGURE_RATIO`.
const RECONFIGURE_RATIO: f64 = 1.3;

fn c_string(text: &str) -> CString {
    CString::new(text).expect("no NUL in GStreamer names")
}

/// Fails with `pipeline`'s error, if it posted one; drops every other message posted since. Nothing else reads its bus,
/// and a bus keeps what it is posted until it is popped: a long-lived pipeline's state, latency, QoS and element
/// messages piled up for as long as the stream ran.
fn check_bus(api: &Api, pipeline: Pointer, what: &str) -> Result<()> {
    // SAFETY: the bus and message references are released here
    unsafe {
        let bus = (api.element_get_bus)(pipeline);
        let message = (api.bus_pop_filtered)(bus, MESSAGE_ERROR);
        loop {
            let other = (api.bus_pop_filtered)(bus, MESSAGE_ANY_BUT_ERROR);
            if other.is_null() {
                break;
            }
            (api.mini_object_unref)(other);
        }
        (api.object_unref)(bus);
        if message.is_null() {
            return Ok(());
        }
        let (mut error, mut debug) = (null_mut(), null_mut());
        (api.message_parse_error)(message, &mut error, &mut debug);
        let details = if debug.is_null() { String::new() } else { CStr::from_ptr(debug).to_string_lossy().into_owned() };
        (api.free)(debug.cast());
        (api.mini_object_unref)(message);
        bail!("GStreamer {what}: {} ({details})", api.take_error(error))
    }
}

/// Builds `description`, finds the elements called `source` and `sink` in it, and starts it: (pipeline, source, sink),
/// owned references.
///
/// The names matter beyond the lookup: GStreamer names a pad's streaming thread after its element, and threads
/// created from it inherit that name — so every pipeline here calling its appsrc `source` meant that a decoder's
/// thread, an encoder's thread and libav's workers all read as `source:src` in `ps`. That cost two wrong conclusions
/// in one day (eight leaked pipelines that were one decoder's workers, then four decode threads that were two decoders
/// and two encoders). Each stage names its own, so a census says what it is looking at.
fn launch(api: &Api, description: &str, source: &str, sink: &str) -> Result<(Pointer, Pointer, Pointer)> {
    let mut error = null_mut();
    // SAFETY: plain GStreamer calls; the caller owns every returned reference
    unsafe {
        let pipeline = (api.parse_launch)(c_string(description).as_ptr(), &mut error);
        if pipeline.is_null() || !error.is_null() {
            let message = api.take_error(error);
            if !pipeline.is_null() {
                (api.object_unref)(pipeline);
            }
            bail!("GStreamer pipeline {description:?}: {message}");
        }
        let by_name = |name: &str| (api.bin_get_by_name)(pipeline, c_string(name).as_ptr());
        let (source, sink) = (by_name(source), by_name(sink));
        if source.is_null() || sink.is_null() || (api.element_set_state)(pipeline, STATE_PLAYING) == 0 {
            for object in [source, sink, pipeline].into_iter().filter(|object| !object.is_null()) {
                (api.object_unref)(object);
            }
            bail!("GStreamer pipeline {description:?} would not start");
        }
        Ok((pipeline, source, sink))
    }
}

/// Pushes `data` into `source` as one buffer.
fn push(api: &Api, source: Pointer, data: &[u8]) -> Result<()> {
    // SAFETY: the buffer is handed over to appsrc, or released on failure
    unsafe {
        let buffer = (api.buffer_new_allocate)(null_mut(), data.len(), null_mut());
        ensure!(!buffer.is_null(), "GStreamer: no buffer");
        let filled = (api.buffer_fill)(buffer, 0, data.as_ptr().cast(), data.len());
        if filled != data.len() {
            (api.mini_object_unref)(buffer);
            bail!("GStreamer: filled {filled} of {} bytes", data.len());
        }
        let flow = (api.app_src_push_buffer)(source, buffer);
        ensure!(flow == 0, "GStreamer push: flow {flow}");
        Ok(())
    }
}

/// The next buffer out of `sink`, waiting at most `timeout`; None if none came.
fn pull(api: &Api, sink: Pointer, timeout: Duration) -> Result<Option<Vec<u8>>> {
    // SAFETY: the sample is released once its buffer is copied out
    unsafe {
        let sample = (api.app_sink_try_pull_sample)(sink, timeout.as_nanos() as u64);
        if sample.is_null() {
            return Ok(None);
        }
        let out = (api.sample_get_buffer)(sample);
        let mut data = vec![0u8; (api.buffer_get_size)(out)];
        let extracted = (api.buffer_extract)(out, 0, data.as_mut_ptr().cast(), data.len());
        (api.mini_object_unref)(sample);
        ensure!(extracted == data.len(), "GStreamer: extracted {extracted} of {} bytes", data.len());
        Ok(Some(data))
    }
}

/// A running `appsrc ! … ! appsink` pipeline (owned references, released on drop).
struct Pipeline {
    api: &'static Api,
    pipeline: Pointer,
    source: Pointer,
    sink: Pointer,
    encoder: Pointer,
    element: &'static Element,
    size: (u32, u32),
    /// the JPEG size it decodes, for a pipeline that takes JPEG
    jpeg: Option<(u32, u32)>,
    fps: f64,
    bitrate_bps: u32,
    /// the frame ids inside, oldest first: access units come out in order, one per frame
    inside: VecDeque<u64>,
}

// SAFETY: GStreamer objects are thread-safe; the encoder uses them one call at a time (`&mut self`)
unsafe impl Send for Pipeline {}

impl Drop for Pipeline {
    fn drop(&mut self) {
        // SAFETY: our references, released once
        unsafe {
            (self.api.element_set_state)(self.pipeline, STATE_NULL);
            for object in [self.source, self.sink, self.encoder, self.pipeline] {
                (self.api.object_unref)(object);
            }
        }
    }
}

impl Pipeline {
    fn new(api: &'static Api, element: &'static Element, (width, height): (u32, u32), jpeg: Option<(u32, u32)>, bitrate_bps: u32, fps: f64, ends: &(String, String)) -> Result<Self> {
        let rate = (fps * 1000.0).round().max(1.0) as u64;
        let description = match jpeg {
            // decoded, scaled and encoded without leaving the GPU's memory (queues: each stage on its own thread); on the
            // R1 under load 35 a 1920x1536 frame took 4 ms, against 26-60 decoded and scaled on the CPU and copied in
            Some((jpeg_width, jpeg_height)) => format!(
                "appsrc name={} is-live=true do-timestamp=true format=time caps=image/jpeg,width={jpeg_width},height={jpeg_height},framerate={rate}/1000 ! jpegparse ! queue ! nvjpegdec ! video/x-raw(memory:NVMM) ! queue ! nvvidconv ! video/x-raw(memory:NVMM),format=I420,width={width},height={height} ! queue ! {} name=encoder !{} appsink name={} sync=false",
                ends.0,
                element.name,
                element.output_chain(api),
                ends.1,
            ),
            // colorimetry 2:4:5:1: limited range, BT.601 matrix, BT.709 transfer and primaries (zenoh-gateway's pictures)
            None => format!(
                "appsrc name={} is-live=true do-timestamp=true format=time caps=video/x-raw,format=I420,width={width},height={height},framerate={rate}/1000,colorimetry=2:4:5:1 ! {} {} name=encoder !{} appsink name={} sync=false",
                ends.0,
                element.upload,
                element.name,
                element.output_chain(api),
                ends.1,
            ),
        };
        let mut error = null_mut();
        // SAFETY: plain GStreamer calls; every returned reference is owned by the Pipeline
        unsafe {
            let pipeline = (api.parse_launch)(c_string(&description).as_ptr(), &mut error);
            if pipeline.is_null() || !error.is_null() {
                let message = api.take_error(error);
                if !pipeline.is_null() {
                    (api.object_unref)(pipeline);
                }
                bail!("GStreamer pipeline {description:?}: {message}");
            }
            let by_name = |name: &str| (api.bin_get_by_name)(pipeline, c_string(name).as_ptr());
            let mut pipeline = Pipeline { api, pipeline, source: by_name(&ends.0), sink: by_name(&ends.1), encoder: by_name("encoder"), element, size: (width, height), jpeg, fps, bitrate_bps: 0, inside: VecDeque::new() };
            ensure!(!pipeline.source.is_null() && !pipeline.sink.is_null() && !pipeline.encoder.is_null(), "GStreamer pipeline lost its elements");
            // what the encoder was actually configured with, once per pipeline. A property that does not apply leaves
            // the encoder on its defaults, which for x264enc means `preset=medium` with lookahead and B-frames - tens
            // of times slower than `ultrafast` and seconds of latency - and nothing about the stream says why. So say
            // which landed and which did not, rather than leaving it to be inferred from the frame rate.
            let mut applied = Vec::new();
            let mut missing = Vec::new();
            for (name, value) in element.settings {
                if pipeline.set(name, value) { applied.push(format!("{name}={value}")) } else { missing.push(*name) }
            }
            // the bitrate too: it comes from the allocator's grant, not this table, and a tiny one explains a small
            // picture on its own - the policy shrinks to hold its bits per pixel - without the encoder being at fault
            applied.push(format!("bitrate={}{}", if element.kbps { bitrate_bps.div_ceil(1000) } else { bitrate_bps }, if element.kbps { "kbit/s" } else { "bit/s" }));
            log::info!("GStreamer {}: {}", element.name, applied.join(" "));
            if !missing.is_empty() {
                log::warn!("GStreamer {}: no property {} - it keeps its default there, which for an encoder is usually much slower", element.name, missing.join(", "));
            }
            let interval = ((fps * super::KEYFRAME_SECONDS).ceil() as u32).max(1).to_string();
            for name in element.keyframe_interval {
                pipeline.set(name, &interval);
            }
            pipeline.set_bitrate(bitrate_bps);
            ensure!((api.element_set_state)(pipeline.pipeline, STATE_PLAYING) != 0, "GStreamer pipeline would not start");
            Ok(pipeline)
        }
    }

    /// Sets an encoder property from its string form; `false` if the installed version has no such property, in which
    /// case the encoder keeps its default and the caller should say so.
    fn set(&self, name: &str, value: &str) -> bool {
        let name = c_string(name);
        // SAFETY: a GObject's first field is its class pointer
        unsafe {
            let class = *(self.encoder as *const Pointer);
            if (self.api.class_find_property)(class, name.as_ptr()).is_null() {
                return false;
            }
            (self.api.util_set_object_arg)(self.encoder, name.as_ptr(), c_string(value).as_ptr());
        }
        true
    }

    fn set_bitrate(&mut self, bitrate_bps: u32) {
        let value = if self.element.kbps { bitrate_bps.div_ceil(1000) } else { bitrate_bps };
        self.set("bitrate", &value.to_string());
        self.bitrate_bps = bitrate_bps;
    }

    fn check_bus(&self) -> Result<()> {
        check_bus(self.api, self.pipeline, self.element.name)
    }

    /// Pushes one picture (I420 in GStreamer's layout, or a JPEG) and takes the oldest access unit out: at once if one is
    /// ready, waiting only once [`max_in_flight`] frames are inside (every access unit comes out, in order, one per call).
    /// Returns it with its frame id.
    fn encode(&mut self, buffer: &[u8], keyframe: bool, frame_id: u64) -> Result<Option<(Vec<u8>, u64)>> {
        // SAFETY: the event is handed over to appsrc
        unsafe {
            if keyframe {
                let event = (self.api.force_key_unit)(CLOCK_TIME_NONE, CLOCK_TIME_NONE, CLOCK_TIME_NONE, 1, 0);
                if (self.api.element_send_event)(self.source, event) == 0 {
                    log::warn!("GStreamer {}: the keyframe request was not handled", self.element.name);
                }
            }
        }
        push(self.api, self.source, buffer)?;
        self.inside.push_back(frame_id);
        let in_flight = if self.jpeg.is_some() { jpeg_in_flight() } else { max_in_flight() };
        let timeout = if self.inside.len() as u64 > in_flight { OUTPUT_TIMEOUT } else { Duration::ZERO };
        let data = pull(self.api, self.sink, timeout)?;
        self.check_bus()?;
        Ok(data.map(|data| (data, self.inside.pop_front().unwrap_or(frame_id))))
    }

    /// An access unit that came out since, without pushing, waiting at most `wait` for one, and its frame id.
    fn next(&mut self, wait: Duration) -> Result<Option<(Vec<u8>, u64)>> {
        if self.inside.is_empty() {
            return Ok(None);
        }
        let data = pull(self.api, self.sink, wait)?;
        Ok(data.and_then(|data| Some((data, self.inside.pop_front()?))))
    }
}

/// Packed I420 in GStreamer's default layout for raw caps (rows padded to 4 bytes): read packed, a width that is not a
/// multiple of 8 came out diagonally sheared and off-color (1362 wide through nvv4l2h264enc on the R1).
fn gstreamer_layout(packed: &[u8], (width, height): (u32, u32)) -> Cow<'_, [u8]> {
    let (width, height) = (width as usize, height as usize);
    let (luma_stride, chroma_stride) = (width.next_multiple_of(4), (width / 2).next_multiple_of(4));
    if (luma_stride, chroma_stride) == (width, width / 2) {
        return Cow::Borrowed(packed);
    }
    let (luma, chroma) = packed.split_at(width * height);
    let mut out = Vec::with_capacity(luma_stride * height + chroma_stride * height);
    for (rows, row_len, stride) in [(luma, width, luma_stride), (chroma, width / 2, chroma_stride)] {
        for row in rows.chunks_exact(row_len) {
            out.extend_from_slice(row);
            out.resize(out.len() + stride - row_len, 0);
        }
    }
    Cow::Owned(out)
}

/// Whether an Annex B access unit holds an IDR slice (NAL type 5).
fn has_idr(data: &[u8]) -> bool {
    data.windows(4).any(|window| window[..3] == [0, 0, 1] && window[3] & 0x1f == 5)
}

/// H.264 through one GStreamer encoder element; see [`hardware`](GstreamerEncoder::hardware).
pub struct GstreamerEncoder {
    api: &'static Api,
    element: &'static Element,
    pipeline: Option<Pipeline>,
    /// the frame id of the access unit `encode` last returned
    last_frame_id: Option<u64>,
    /// the names of this encoder's pipeline ends, kept across the rebuilds a size change costs so that the thread
    /// doing its work keeps the same name ([`ends`])
    ends: (String, String),
}

impl GstreamerEncoder {
    /// An encoder using the element called `name` (one of the hardware ones, or `x264enc`), if GStreamer has it.
    pub fn with_element(name: &str) -> Result<Self> {
        let api = api()?;
        let element = HARDWARE.iter().chain([&X264]).find(|element| element.name == name).ok_or_else(|| anyhow!("{name} is not an encoder this crate drives"))?;
        // SAFETY: the factory reference is released at once
        unsafe {
            let factory = (api.element_factory_find)(c_string(name).as_ptr());
            ensure!(!factory.is_null(), "GStreamer has no {name}");
            (api.object_unref)(factory);
        }
        Ok(GstreamerEncoder { api, element, pipeline: None, last_frame_id: None, ends: ends("enc", next_number(&ENCODERS)) })
    }

    /// The first hardware encoder that is installed and encodes a test frame.
    pub fn hardware() -> Result<Self> {
        let mut tried = Vec::new();
        for element in &HARDWARE {
            match GstreamerEncoder::with_element(element.name).and_then(|mut encoder| encoder.probe().map(|()| encoder)) {
                Ok(encoder) => return Ok(GstreamerEncoder { pipeline: None, ..encoder }),
                Err(error) => tried.push(format!("{}: {error:#}", element.name)),
            }
        }
        bail!("no GStreamer hardware H.264 encoder works ({})", tried.join("; "))
    }

    /// The element in use, e.g. `"nvv4l2h264enc"`.
    pub fn element(&self) -> &'static str {
        self.element.name
    }

    fn probe(&mut self) -> Result<()> {
        let (frame, target) = super::probe_frame()?;
        for _ in 0..=max_in_flight() + 1 {
            if self.encode(&frame, &target)?.is_some() {
                return Ok(());
            }
        }
        bail!("no frame came out")
    }
}

impl VideoEncoder for GstreamerEncoder {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn last_frame_id(&self) -> Option<u64> {
        self.last_frame_id
    }

    fn next_output(&mut self, wait: Duration) -> Result<Option<EncodedVideo>> {
        let Some(pipeline) = self.pipeline.as_mut() else { return Ok(None) };
        let Some((data, frame_id)) = pipeline.next(wait)? else { return Ok(None) };
        self.last_frame_id = Some(frame_id);
        Ok(Some(EncodedVideo { keyframe: has_idr(&data), data, width: pipeline.size.0, height: pipeline.size.1 }))
    }

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        let DecodedFrame::Video(image) = frame else { bail!("{} takes pictures, got {frame:?}", self.element.name) };
        let size = at_least((target.width, target.height), self.element.min_size);
        // encoders budget each frame from the caps' rate, so a rate that moves far rebuilds the pipeline
        // a JPEG the Jetson's decoder takes goes in as it is (see `Pipeline::new`)
        let jpeg = image.jpeg_bytes().filter(|_| self.element.name == JPEG_ENCODER).and_then(|bytes| Some((bytes, crate::hardware_jpeg::hardware_size(bytes)?)));
        let fresh = self.pipeline.as_ref().is_none_or(|pipeline| pipeline.size != size || pipeline.jpeg != jpeg.map(|(_, jpeg_size)| jpeg_size) || (pipeline.fps / target.fps).max(target.fps / pipeline.fps) > RECONFIGURE_RATIO);
        if fresh {
            self.pipeline = Some(Pipeline::new(self.api, self.element, size, jpeg.map(|(_, jpeg_size)| jpeg_size), target.bitrate_bps, target.fps, &self.ends)?);
            log::info!("GStreamer {} encoding {}x{} on thread {}:src", self.element.name, size.0, size.1, self.ends.0);
        }
        let pipeline = self.pipeline.as_mut().expect("created above");
        if pipeline.bitrate_bps != target.bitrate_bps {
            pipeline.set_bitrate(target.bitrate_bps);
        }
        let encoded = match jpeg {
            Some((bytes, _)) => pipeline.encode(bytes, target.keyframe && !fresh, target.frame_id)?,
            None => pipeline.encode(&gstreamer_layout(image.to_i420(size.0, size.1)?.data(), size), target.keyframe && !fresh, target.frame_id)?,
        };
        self.last_frame_id = encoded.as_ref().map(|(_, frame_id)| *frame_id);
        let encoded = encoded.map(|(data, _)| data);
        Ok(encoded.map(|data| EncodedVideo { keyframe: has_idr(&data), data, width: size.0, height: size.1 }))
    }
}

/// The Jetson's hardware JPEG decoder: `nvjpegdec`, then `nvvidconv` to packed I420 (it takes JFIF's full range to the
/// limited range zenoh-gateway's pictures are in). On the R1's Orin, under load 29, a 1920x1536 camera frame cost 9 ms
/// of CPU this way against 16 with libjpeg-turbo. It failed on a 4:4:4 JPEG; the R1's cameras send 4:2:0.
pub struct JpegDecoder {
    api: &'static Api,
    /// (pipeline, appsrc, appsink, the size it was made for)
    pipeline: Option<(Pointer, Pointer, Pointer, (u32, u32))>,
    /// the names of its pipeline's ends ([`ends`])
    ends: (String, String),
}

// SAFETY: GStreamer objects are thread-safe; the decoder uses them one call at a time (`&mut self`)
unsafe impl Send for JpegDecoder {}

impl Drop for JpegDecoder {
    fn drop(&mut self) {
        self.stop();
    }
}

impl JpegDecoder {
    /// The hardware decoder, if GStreamer has `nvjpegdec`.
    pub fn hardware() -> Result<Self> {
        let api = api()?;
        // SAFETY: the factory reference is released at once
        unsafe {
            let factory = (api.element_factory_find)(c_string("nvjpegdec").as_ptr());
            ensure!(!factory.is_null(), "GStreamer has no nvjpegdec");
            (api.object_unref)(factory);
        }
        Ok(JpegDecoder { api, pipeline: None, ends: ends("jpeg", next_number(&JPEG_DECODERS)) })
    }

    fn stop(&mut self) {
        if let Some((pipeline, source, sink, _)) = self.pipeline.take() {
            // SAFETY: our references, released once
            unsafe {
                (self.api.element_set_state)(pipeline, STATE_NULL);
                for object in [source, sink, pipeline] {
                    (self.api.object_unref)(object);
                }
            }
        }
    }

    /// A `width x height` (both even) JPEG as packed I420, waiting for it (one pipeline per size).
    pub fn decode(&mut self, jpeg: &[u8], (width, height): (u32, u32)) -> Result<Vec<u8>> {
        if self.pipeline.is_none_or(|(_, _, _, size)| size != (width, height)) {
            self.stop();
            let description = format!("appsrc name={} is-live=true format=time caps=image/jpeg,width={width},height={height},framerate=0/1 ! jpegparse ! nvjpegdec ! nvvidconv ! video/x-raw,format=I420,width={width},height={height} ! appsink name={} sync=false", self.ends.0, self.ends.1);
            let (pipeline, source, sink) = launch(self.api, &description, &self.ends.0, &self.ends.1)?;
            self.pipeline = Some((pipeline, source, sink, (width, height)));
        }
        let (pipeline, source, sink, _) = self.pipeline.expect("made above");
        push(self.api, source, jpeg)?;
        let data = pull(self.api, sink, OUTPUT_TIMEOUT)?;
        check_bus(self.api, pipeline, "nvjpegdec")?;
        let data = data.with_context(|| format!("nvjpegdec: no picture within {OUTPUT_TIMEOUT:?}"))?;
        packed_i420(data, (width, height))
    }
}

/// The element an [`H265Decoder`] decodes with unless `ZDC_H265_GST_DECODER` names another: libav's HEVC decoder,
/// which GStreamer wraps. On an RK3588 A76 core it does ~91 fps of 720p against libde265's ~50, and libde265 on the
/// little cores managed 14-18 — software H.265 is only fast enough here with this one.
///
/// Set it to `mppvideodec` where the Rockchip VPU is reachable (`librockchip_mpp`, `/dev/mpp_service`): the pipeline is
/// otherwise identical, which is the point of naming the element rather than hard-coding it.
const DEFAULT_H265_DECODER: &str = "avdec_h265";

/// Decoders whose sink caps ask for `parsed=true`, which only an `h265parse` upstream sets.
///
/// **An RK3588's `mppvideodec` is not yet usable through this and needs one more thing tried.** With the parser in
/// front of it, it still fails at the input with `Internal data stream error` from appsrc. Its sink caps were reported
/// as `video/x-h265, parsed:true, alignment:au` — with **no `stream-format`**, which is the signature of a hardware
/// decoder that wants `hvc1`/`hev1` (length-prefixed, parameter sets in `codec_data`) rather than the `byte-stream`
/// this feeds. The next thing to try is therefore a caps filter converting it, which `h265parse` can do:
///
/// ```text
/// appsrc ! h265parse ! video/x-h265,stream-format=hvc1,alignment=au ! mppvideodec ! videoconvert ! ...
/// ```
///
/// That would be a third shape for [`H265Decoder::description`] rather than another entry here, since it differs in
/// the caps between the parser and the decoder and not in whether a parser is present. Two attempts have been spent
/// on this already (the decoder's NV12 output, then `parsed=true`), so it is worth knowing before a third: the
/// software path is latency-bound, not CPU-bound, and pipelining the decoder fixes that portably on any machine,
/// where this only ever helps a Rockchip.
///
/// A hardware decoder generally wants it; libav's takes byte-stream access units as they are. Feeding one of these
/// without the parser fails with `Internal data stream error` from appsrc, which looks like the hardware refusing the
/// stream rather than a missing element - so the list is explicit, and an element not on it keeps the parser-free
/// pipeline that libav's is known to work with. A decoder that needs the parser and is not named here shows that same
/// error; add it.
const PARSED_INPUT_DECODERS: &[&str] = &["mppvideodec", "v4l2h265dec", "nvv4l2decoder", "vah265dec", "vaapih265dec"];

/// Decoder pipelines alive now, and how many have ever been made: whether a resync leaves the old pipeline behind is
/// a question a thread count cannot answer (libav names its worker threads after the thread that spawned them, so one
/// pipeline shows one `source:src` thread per core), so the decoder says so itself.
static PIPELINES: (AtomicU64, AtomicU64) = (AtomicU64::new(0), AtomicU64::new(0));

/// Where a decode call's wall clock goes, reported every [`DECODE_REPORT`] calls.
///
/// The gateway measured 31.5 ms a frame for a stream whose raw serial decode on the same core is 11.1 ms, and no
/// counter could say where the other 20 ms went. These are the parts of the call, in the order it runs them: the work
/// is memory as much as it is the codec (a 720p picture is 1.38 MB, and copying it three times costs more than
/// decoding it on some machines), so each is timed rather than reasoned about.
#[derive(Default)]
struct Timings {
    calls: u64,
    /// handing the access unit to appsrc (a buffer allocation and a copy of the compressed frame)
    push: Duration,
    /// taking the picture out: the decode itself happens on the pipeline's own thread, so this is the part of it that
    /// was not already done when we asked
    pull: Duration,
    /// reading the pipeline's error bus
    bus: Duration,
    /// repacking GStreamer's padded rows into packed I420, where its stride is not the picture's width
    repack: Duration,
}

impl Timings {
    fn line(&self, element: &str) -> String {
        let each = |part: Duration| part.as_secs_f64() * 1000.0 / self.calls.max(1) as f64;
        format!(
            "{element}: {} calls, each: push {:.2}, pull {:.2}, bus {:.2}, repack {:.2} ms",
            self.calls,
            each(self.push),
            each(self.pull),
            each(self.bus),
            each(self.repack)
        )
    }
}

/// Decode calls between timing lines: ~5 s at 30 Hz.
const DECODE_REPORT: u64 = 150;

/// The appsrc and appsink names a pipeline uses, which is also what its streaming thread is called: `h265src0:src` at
/// 80% of a core says which stage **and which stream** that is, where four threads all called `source:src` said
/// nothing. Numbered per decoder or encoder, because two cameras have one of each and they have to be told apart to be
/// placed on different cores; the number is logged with the key when the pipeline starts.
///
/// Linux keeps 15 characters of a thread name and GStreamer appends `:src`, so these stay short.
fn ends(stage: &str, number: u64) -> (String, String) {
    (format!("{stage}src{number}"), format!("{stage}sink{number}"))
}

/// Numbers for [`ends`], one sequence per stage.
fn next_number(counter: &std::sync::atomic::AtomicU64) -> u64 {
    counter.fetch_add(1, Ordering::Relaxed)
}

static H265_DECODERS: AtomicU64 = AtomicU64::new(0);
static ENCODERS: AtomicU64 = AtomicU64::new(0);
static JPEG_DECODERS: AtomicU64 = AtomicU64::new(0);

/// (live, ever made) decoder pipelines. One per stream is right; one more per resync was the shape of a bug.
pub fn decoder_pipelines() -> (u64, u64) {
    (PIPELINES.0.load(Ordering::Relaxed), PIPELINES.1.load(Ordering::Relaxed))
}

/// H.265 decoded by GStreamer, keeping its state between access units so predicted frames decode.
///
/// [`h265::Stream`](crate::utils::h265::Stream) does the same job with libde265, which is the fallback where GStreamer
/// is not installed; this is several times faster and is what a robot should use. Both are fed the same way — every
/// access unit of one stream, in order — so they sit behind one `StreamDecoder`.
///
/// No parser in the pipeline: the access units handed in are already whole and carry their parameter sets
/// ([`decodable`](crate::utils::compressed_video::decodable)), so `h265parse` would have nothing to do, and leaving it
/// out means this works where `gstreamer1.0-plugins-bad` is not installed. The caps say `alignment=au` for that reason;
/// `alignment=nal` is refused by the decoder.
pub struct H265Decoder {
    api: &'static Api,
    /// the element with any properties, as it goes into the pipeline
    element: String,
    /// just the factory, for the messages and for `PARSED_INPUT_DECODERS`
    factory: String,
    /// (pipeline, appsrc, appsink); made on the first access unit, since the pipeline is not size-specific
    pipeline: Option<(Pointer, Pointer, Pointer)>,
    /// the frame ids of the units inside, oldest first: pictures come out in the order the units went in, one per unit
    /// (as the encoder's `inside` assumes of its access units). A stream whose pictures were *reordered* — HEVC with a
    /// nonzero reorder depth, which the robot's camera does not use (`ffprobe has_b_frames=0`) — would pair a picture
    /// with a neighbouring sample's stamp. Fixing that needs the id back out of GStreamer with the buffer, and
    /// `gst_buffer_get_pts` is a macro rather than a symbol this binding can dlopen
    inside: VecDeque<u64>,
    /// how many units may be inside before `decode` waits for a picture ([`frames_inside`])
    inside_max: u64,
    /// samples were lost: nothing goes in until a keyframe, which is what makes the picture whole again
    after_gap: bool,
    /// where the calls' wall clock goes ([`Timings`])
    timings: Timings,
    /// the names of its pipeline's ends, kept across a rebuild so the thread doing its work keeps its name ([`ends`])
    ends: (String, String),
    /// pipelines this decoder has built: one is right, and one per lost sample was the shape of a bug
    made: u64,
}

// SAFETY: GStreamer objects are thread-safe; the decoder uses them one call at a time (`&mut self`)
unsafe impl Send for H265Decoder {}

impl Drop for H265Decoder {
    fn drop(&mut self) {
        self.stop();
    }
}

impl H265Decoder {
    /// A decoder using `ZDC_H265_GST_DECODER` or [`DEFAULT_H265_DECODER`], if GStreamer has that element.
    pub fn new() -> Result<Self> {
        let api = api()?;
        let element = std::env::var("ZDC_H265_GST_DECODER").unwrap_or_else(|_| DEFAULT_H265_DECODER.to_owned());
        // the first word is the factory; what follows is properties for it (`ZDC_H265_GST_DECODER="avdec_h265
        // thread-type=slice"`), which go into the pipeline as they are — so a decoder's threading can be tried on a
        // robot without a build
        let factory_name = element.split_whitespace().next().unwrap_or_default().to_owned();
        // SAFETY: the factory reference is released at once
        unsafe {
            let factory = (api.element_factory_find)(c_string(&factory_name).as_ptr());
            ensure!(!factory.is_null(), "GStreamer has no {factory_name}");
            (api.object_unref)(factory);
        }
        Ok(H265Decoder { api, element, factory: factory_name, pipeline: None, inside: VecDeque::new(), inside_max: frames_inside(), after_gap: false, made: 0, timings: Timings::default(), ends: ends("h265", next_number(&H265_DECODERS)) })
    }

    /// The element in use, e.g. `"avdec_h265"`.
    pub fn element(&self) -> &str {
        &self.element
    }

    /// Pipelines **this** decoder has built. One is right: a lost sample is not a reason to build another, and a
    /// count that climbs with [`resync`](Self::resync) is the ~35 ms-per-gap stall coming back.
    pub fn pipelines_made(&self) -> u64 {
        self.made
    }

    /// How many access units may be inside before [`decode`](Self::decode) waits for a picture, over
    /// [`frames_inside`]. `0` waits for every picture, which is one decode and one encode in series per frame — what
    /// capped a 720p stream at ~22 fps, and what a throughput measurement wants, since then every picture comes out.
    pub fn hold(&mut self, frames: u64) {
        self.inside_max = frames;
    }

    fn stop(&mut self) {
        if let Some((pipeline, source, sink)) = self.pipeline.take() {
            // SAFETY: our references, released once
            unsafe {
                (self.api.element_set_state)(pipeline, STATE_NULL);
                for object in [source, sink, pipeline] {
                    (self.api.object_unref)(object);
                }
            }
            let live = PIPELINES.0.fetch_sub(1, Ordering::Relaxed) - 1;
            log::info!("H.265 decoder pipeline stopped: {live} live, {} made", PIPELINES.1.load(Ordering::Relaxed));
        }
        // the units inside went with the pipeline
        self.inside.clear();
    }

    /// Throws the pipeline away, so the next access unit starts a new one. For a stream the decoder could not follow
    /// (a bus error): everything after it may be built on what went wrong.
    ///
    /// **Not what a lost sample needs** — that is [`resync`](Self::resync), which keeps the pipeline. Rebuilding on
    /// every lost sample cost ~35 ms of stall each, and a stream that loses one sample every few seconds spends that
    /// again and again on a decoder that was working.
    pub fn reset(&mut self) {
        self.stop();
    }

    /// Samples were lost: show nothing until the next keyframe, **keeping this pipeline**.
    ///
    /// A keyframe is enough on its own — an IRAP picture clears the decoder's reference pictures by definition, which
    /// is the whole of what a new pipeline would have given. The units already pushed are left inside: their pictures
    /// were decoded from samples that did arrive, they still come out in order, and the frame ids keep them paired.
    pub fn resync(&mut self) {
        self.after_gap = true;
    }

    /// The pipeline for `element`, with `h265parse` in front of it or without.
    ///
    /// `framerate=0/1`: the decoder is driven by what is pushed, not by a clock.
    ///
    /// `videoconvert` because a decoder gives whichever layout it likes and the gateway's pictures are I420: libav's
    /// gives I420 already and the element is then a passthrough costing nothing, while a hardware one gives NV12 and
    /// would otherwise refuse to negotiate. Where the conversion does run, a hardware one in front of it
    /// (`rgaconvert`) keeps it off the CPU.
    ///
    /// A pure function so both shapes can be checked on a machine that has neither element - the parser-free one is
    /// all that a machine without `gstreamer1.0-plugins-bad` can ever run, and an unparsed hardware pipeline is how a
    /// day was lost once already.
    fn description(element: &str, parse: bool, ends: &(String, String)) -> String {
        let parser = if parse { " h265parse !" } else { "" };
        format!(
            "appsrc name={} is-live=true format=time caps=video/x-h265,stream-format=byte-stream,alignment=au,framerate=0/1 !{parser} {element} ! videoconvert ! video/x-raw,format=I420 ! appsink name={} sync=false",
            ends.0, ends.1
        )
    }

    /// Whether the next access unit has to be a keyframe, because there is no pipeline yet or the last one was thrown
    /// away. **Check this before pushing**: a fresh pipeline fed a predicted frame produces nothing, and `decode` then
    /// waits [`OUTPUT_TIMEOUT`] for that nothing — seconds, per frame, for as long as the keyframes are missed. That
    /// turns a stream that should drop a second of pictures after a loss into one that delivers almost none.
    pub fn needs_keyframe(&self) -> bool {
        self.pipeline.is_none() || self.after_gap
    }

    /// Feeds one Annex-B access unit, which follows every unit fed before it, and takes the oldest picture out as
    /// packed I420, with the `frame_id` of the unit **that picture** came from: at once if one is ready, waiting only
    /// once more than [`frames_inside`] units are inside. `None`: nothing out yet (no keyframe seen yet, a unit that
    /// completes no picture, or a picture still decoding).
    ///
    /// `size` is the picture's size, which the caller already knows from the stream's sequence parameter set
    /// ([`h265_sps::picture_size`](crate::utils::h265_sps::picture_size)) — cheaper and more direct than reading the
    /// sink's negotiated caps back out of GStreamer, and `packed_i420` fails loudly if the decoder disagrees.
    pub fn decode(&mut self, access_unit: &[u8], size: (u32, u32), frame_id: u64) -> Result<Option<(Vec<u8>, u64)>> {
        // a keyframe is what a gap was waiting for: from here the pictures are whole again
        self.after_gap = false;
        if self.pipeline.is_none() {
            let parse = PARSED_INPUT_DECODERS.contains(&self.factory.as_str());
            if parse {
                ensure!(has_element(self.api, "h265parse"), "{} needs h265parse in front of it (gstreamer1.0-plugins-bad), which GStreamer here does not have", self.factory);
            }
            let description = Self::description(&self.element, parse, &self.ends);
            let (pipeline, source, sink) = launch(self.api, &description, &self.ends.0, &self.ends.1)?;
            self.pipeline = Some((pipeline, source, sink));
            self.made += 1;
            let (live, made) = (PIPELINES.0.fetch_add(1, Ordering::Relaxed) + 1, PIPELINES.1.fetch_add(1, Ordering::Relaxed) + 1);
            // one per stream is right; one per resync is the bug this count exists to show
            log::info!("H.265 decoder pipeline started ({}) on thread {}:src: {live} live, {made} made", self.element, self.ends.0);
        }
        let (pipeline, source, sink) = self.pipeline.expect("made above");
        let part = Instant::now();
        push(self.api, source, access_unit)?;
        self.inside.push_back(frame_id);
        self.timings.calls += 1;
        self.timings.push += part.elapsed();
        let waited = self.inside.len() as u64 > self.inside_max;
        let part = Instant::now();
        let data = pull(self.api, sink, if waited { OUTPUT_TIMEOUT } else { Duration::ZERO })?;
        self.timings.pull += part.elapsed();
        let part = Instant::now();
        let bus = check_bus(self.api, pipeline, &self.element);
        self.timings.bus += part.elapsed();
        if self.timings.calls >= DECODE_REPORT {
            log::info!("H.265 decode {}", self.timings.line(&self.element));
            self.timings = Timings::default();
        }
        if let Err(error) = bus {
            // the stream is not what the decoder expected: everything after this may be built on it
            self.reset();
            return Err(error);
        }
        let Some(data) = data else {
            if waited {
                // every access unit here holds one coded picture, so one comes out per unit - except for a unit the
                // decoder could make nothing of. Its id is dropped here rather than left to be handed to the next
                // picture, which would misstamp every picture of the stream from then on
                log::debug!("GStreamer {}: no picture from access unit {:?} within {OUTPUT_TIMEOUT:?}", self.element, self.inside.front());
                self.inside.pop_front();
            }
            return Ok(None);
        };
        // the oldest unit inside is the one this picture came from
        let frame_id = self.inside.pop_front().unwrap_or(frame_id);
        let part = Instant::now();
        let picture = packed_i420(data, size)?;
        self.timings.repack += part.elapsed();
        Ok(Some((picture, frame_id)))
    }
}

/// GStreamer's default I420 layout (rows padded to 4 bytes) as packed I420.
fn packed_i420(data: Vec<u8>, (width, height): (u32, u32)) -> Result<Vec<u8>> {
    let (width, height) = (width as usize, height as usize);
    let (luma_stride, chroma_stride) = (width.next_multiple_of(4), (width / 2).next_multiple_of(4));
    if data.len() == width * height * 3 / 2 {
        return Ok(data);
    }
    ensure!(data.len() == luma_stride * height + chroma_stride * height, "nvjpegdec: {} bytes for a {width}x{height} I420 picture", data.len());
    let mut out = Vec::with_capacity(width * height * 3 / 2);
    let (luma, chroma) = data.split_at(luma_stride * height);
    for (rows, row_len, stride) in [(luma, width, luma_stride), (chroma, width / 2, chroma_stride)] {
        for row in rows.chunks_exact(stride) {
            out.extend_from_slice(&row[..row_len]);
        }
    }
    Ok(out)
}

/// `size` scaled up (shape kept, width a multiple of 16 and height even, as zenoh-gateway sizes them) to at least `min`.
fn at_least((width, height): (u32, u32), (min_width, min_height): (u32, u32)) -> (u32, u32) {
    let scale = (min_width as f64 / width.max(1) as f64).max(min_height as f64 / height.max(1) as f64);
    if scale <= 1.0 {
        return (width, height);
    }
    let up = |value: u32, step: u32| ((value as f64 * scale).ceil() as u32).next_multiple_of(step);
    (up(width, 16).max(min_width), up(height, 2).max(min_height))
}

#[cfg(test)]
mod tests {
    /// The decoder's own parts, as the line reports them. `pull` is the one that matters: it is the part of the decode
    /// that was **not** already done when the picture was asked for, so a large `pull` is a decoder that is not
    /// keeping up (or not being scheduled), while a large `push` or `repack` is work this code is doing itself.
    #[test]
    fn the_decode_timing_line_separates_waiting_from_working() {
        let timings = super::Timings {
            calls: 150,
            push: std::time::Duration::from_micros(3_000),
            pull: std::time::Duration::from_millis(39),
            bus: std::time::Duration::from_micros(150),
            repack: std::time::Duration::from_micros(0),
        };
        assert_eq!(timings.line("avdec_h265"), "avdec_h265: 150 calls, each: push 0.02, pull 0.26, bus 0.00, repack 0.00 ms");
        assert!(!super::Timings::default().line("avdec_h265").contains("NaN"));
    }

    use super::*;

    #[test]
    fn small_pictures_scale_up_to_the_least_size() {
        assert_eq!(at_least((320, 180), (160, 64)), (320, 180));
        assert_eq!(at_least((128, 72), (160, 64)), (160, 90));
        assert_eq!(at_least((640, 48), (160, 64)), (864, 64));
        assert_eq!(at_least((48, 640), (160, 64)), (160, 2134));
        assert_eq!(at_least((96, 54), (0, 0)), (96, 54));
    }

    /// On a machine with a hardware encoder (else it passes untested): pictures under its least size still encode.
    #[test]
    fn tiny_pictures_encode_in_hardware() {
        let Ok(mut encoder) = GstreamerEncoder::hardware() else { return };
        for (width, height) in [(128, 72), (64, 36), (640, 48), (320, 180)] {
            let image = zenoh_gateway::VideoImage::i420(width, height, vec![128; (width * height * 3 / 2) as usize]).unwrap();
            let target = VideoTarget::new(width, height, 300_000, 30.0);
            let out = (0..=max_in_flight() + 1).find_map(|_| encoder.encode(&DecodedFrame::Video(image.clone()), &target).unwrap());
            let out = out.unwrap_or_else(|| panic!("{width}x{height}: no frame came out"));
            assert!(out.width >= encoder.element.min_size.0 && out.height >= encoder.element.min_size.1, "{width}x{height} came out {}x{}", out.width, out.height);
        }
    }

    #[test]
    fn idr_slices_are_keyframes() {
        assert!(has_idr(&[0, 0, 0, 1, 0x67, 0x42, 0, 0, 0, 1, 0x65, 0x88]), "SPS then an IDR slice");
        assert!(!has_idr(&[0, 0, 0, 1, 0x41, 0x9a]), "a non-IDR slice");
        assert!(!has_idr(&[]));
    }

    #[test]
    fn padded_i420_unpacks() {
        let packed: Vec<u8> = (0..(6 * 2 * 3 / 2) as u8).collect();
        assert_eq!(packed_i420(gstreamer_layout(&packed, (6, 2)).into_owned(), (6, 2)).unwrap(), packed);
        assert_eq!(packed_i420(packed.clone(), (6, 2)).unwrap(), packed, "already packed");
        assert!(packed_i420(vec![0; 5], (6, 2)).is_err());
    }

    #[test]
    fn rows_padded_to_gstreamer_strides() {
        let packed: Vec<u8> = (0..6 * 2 * 3 / 2).map(|index| index as u8 + 1).collect();
        // 6 wide: luma rows to 8 bytes, chroma rows (3) to 4
        assert_eq!(&*gstreamer_layout(&packed, (6, 2)), &[1, 2, 3, 4, 5, 6, 0, 0, 7, 8, 9, 10, 11, 12, 0, 0, 13, 14, 15, 0, 16, 17, 18, 0]);
        let aligned = vec![0u8; 16 * 2 * 3 / 2];
        assert!(matches!(gstreamer_layout(&aligned, (16, 2)), Cow::Borrowed(_)), "a width that is a multiple of 8 is already laid out");
    }

    /// The hardware decode pipeline cannot be run on a machine with neither `mppvideodec` nor `h265parse`, so its
    /// shape is checked here instead - the same gap that let a trailing comma into the encoder's caps and cost a
    /// deploy.
    #[test]
    fn both_decode_pipelines_are_well_formed() {
        // the seventh decoder in a process: its ends carry the number so two cameras can be told apart
        let seventh = super::ends("h265", 7);
        let parser_free = H265Decoder::description("avdec_h265", false, &seventh);
        let parsed = H265Decoder::description("mppvideodec", true, &seventh);

        assert!(!parser_free.contains("h265parse"), "libav's decoder takes access units as they are: {parser_free}");
        assert!(parsed.contains("! h265parse ! mppvideodec !"), "a hardware decoder needs the stream marked parsed: {parsed}");
        for description in [&parser_free, &parsed] {
            // every element separated by exactly one `!`, and no empty stage between two of them
            assert!(!description.contains("!!") && !description.contains("! !"), "an empty stage in {description}");
            for stage in description.split('!') {
                assert!(!stage.trim().is_empty(), "an empty stage in {description}");
            }
            // and the shape the rest of the decoder relies on either way
            assert!(description.contains("alignment=au"), "mppvideodec and avdec both refuse alignment=nal: {description}");
            assert!(description.contains("videoconvert ! video/x-raw,format=I420"), "the gateway's pictures are I420: {description}");
            // named for this stage and this stream: a thread called `h265src0:src` in a census says which stage and
            // which camera it is, where everything called `source:src` said nothing and cost two wrong conclusions
            assert!(description.contains("name=h265src7") && description.contains("name=h265sink7"), "{description}");
        }
        // every decoder known to need the parser is named, so none of them silently gets the parser-free pipeline
        assert!(PARSED_INPUT_DECODERS.contains(&"mppvideodec"), "the RK3588's decoder is the one this was found with");
        assert!(!PARSED_INPUT_DECODERS.contains(&DEFAULT_H265_DECODER), "libav's must keep the pipeline it is known to work with");
    }

    /// Caps GStreamer cannot parse are a pipeline that cannot be built, and the encoder then falls back silently.
    /// `profile=constrained-baseline,` with nothing after it is exactly that, and it only appeared in the branch
    /// taken by a machine that has `h264parse` - so no test on a machine without it could see it.
    #[test]
    fn the_output_chain_is_well_formed_with_and_without_the_parser() {
        let profiled = Element { profile: H264_SDP_PROFILE, name: "x264enc", upload: "", kbps: true, settings: &[], keyframe_interval: &[], min_size: (0, 0) };
        let plain = Element { profile: "", ..profiled };
        for element in [&profiled, &plain] {
            for parser in [true, false] {
                let chain = element.output_chain_with(parser);
                // a field name and a value on either side of every comma: no empty fields, however the parts combine
                for caps in chain.split('!').map(str::trim).filter(|part| part.starts_with("video/x-h264")) {
                    for field in caps.trim_start_matches("video/x-h264").split(',').skip(1) {
                        assert!(field.contains('='), "empty or valueless caps field {field:?} in {chain:?}");
                    }
                }
                assert!(!chain.contains(",,") && !chain.contains(", "), "malformed caps in {chain:?}");
                assert_eq!(chain.contains("h264parse"), parser, "{chain:?}");
                assert_eq!(chain.contains(H264_SDP_PROFILE), !element.profile.is_empty(), "{chain:?}");
                // whatever the shape, the appsink is handed byte-stream access units
                assert!(chain.contains("stream-format=byte-stream,alignment=au"), "{chain:?}");
            }
        }
    }
}
