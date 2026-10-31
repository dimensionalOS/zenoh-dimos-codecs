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
use std::time::Duration;
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
    },
    Element { name: "nvh264enc", upload: "", kbps: true, settings: &[("rc-mode", "cbr"), ("preset", "low-latency-hp"), ("zerolatency", "true"), ("bframes", "0")], keyframe_interval: &["gop-size"], min_size: (0, 0) },
    Element { name: "vah264enc", upload: "videoconvert ! video/x-raw,format=NV12 !", kbps: true, settings: &[("rate-control", "cbr"), ("b-frames", "0"), ("target-usage", "7")], keyframe_interval: &["key-int-max"], min_size: (0, 0) },
    Element { name: "vaapih264enc", upload: "videoconvert ! video/x-raw,format=NV12 !", kbps: true, settings: &[("rate-control", "cbr"), ("max-bframes", "0")], keyframe_interval: &["keyframe-period"], min_size: (0, 0) },
];
/// Software, for trying the pipeline on a machine without a hardware encoder.
static X264: Element = Element { name: "x264enc", upload: "", kbps: true, settings: &[("tune", "zerolatency"), ("speed-preset", "ultrafast"), ("bframes", "0")], keyframe_interval: &["key-int-max"], min_size: (0, 0) };
/// The encoder whose pipeline takes JPEG (the Jetson's: nvjpegdec and nvvidconv in front of it).
pub(crate) const JPEG_ENCODER: &str = "nvv4l2h264enc";
/// Frames inside the encoder before `encode` waits for one to come out: hardware encoders take a few frame times
/// (nvv4l2h264enc ~40 ms on a loaded Orin), so waiting for each frame's own output capped a stream at ~25 fps.
const MAX_IN_FLIGHT: u64 = 3;
/// The same for a pipeline that takes JPEG: its decoder, converter and encoder each hold frames (on the R1, ~40 ms from
/// JPEG to H.264 idle, longer under load), and waiting for each output at 3 inside cost ~35 ms a frame, which zenoh-gateway's
/// CPU governor took for encoding cost and shrank the picture for. `ZDC_GST_JPEG_IN_FLIGHT` overrides it.
fn jpeg_in_flight() -> u64 {
    static IN_FLIGHT: OnceLock<u64> = OnceLock::new();
    *IN_FLIGHT.get_or_init(|| std::env::var("ZDC_GST_JPEG_IN_FLIGHT").ok().and_then(|value| value.parse().ok()).unwrap_or(6))
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

/// Builds `description`, finds its `source` and `sink`, and starts it: (pipeline, source, sink), owned references.
fn launch(api: &Api, description: &str) -> Result<(Pointer, Pointer, Pointer)> {
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
        let (source, sink) = (by_name("source"), by_name("sink"));
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
    fn new(api: &'static Api, element: &'static Element, (width, height): (u32, u32), jpeg: Option<(u32, u32)>, bitrate_bps: u32, fps: f64) -> Result<Self> {
        let rate = (fps * 1000.0).round().max(1.0) as u64;
        let description = match jpeg {
            // decoded, scaled and encoded without leaving the GPU's memory (queues: each stage on its own thread); on the
            // R1 under load 35 a 1920x1536 frame took 4 ms, against 26-60 decoded and scaled on the CPU and copied in
            Some((jpeg_width, jpeg_height)) => format!(
                "appsrc name=source is-live=true do-timestamp=true format=time caps=image/jpeg,width={jpeg_width},height={jpeg_height},framerate={rate}/1000 ! jpegparse ! queue ! nvjpegdec ! video/x-raw(memory:NVMM) ! queue ! nvvidconv ! video/x-raw(memory:NVMM),format=I420,width={width},height={height} ! queue ! {} name=encoder ! h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au ! appsink name=sink sync=false",
                element.name
            ),
            // colorimetry 2:4:5:1: limited range, BT.601 matrix, BT.709 transfer and primaries (zenoh-gateway's pictures)
            None => format!(
                "appsrc name=source is-live=true do-timestamp=true format=time caps=video/x-raw,format=I420,width={width},height={height},framerate={rate}/1000,colorimetry=2:4:5:1 ! {} {} name=encoder ! h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au ! appsink name=sink sync=false",
                element.upload,
                element.name
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
            let mut pipeline = Pipeline { api, pipeline, source: by_name("source"), sink: by_name("sink"), encoder: by_name("encoder"), element, size: (width, height), jpeg, fps, bitrate_bps: 0, inside: VecDeque::new() };
            ensure!(!pipeline.source.is_null() && !pipeline.sink.is_null() && !pipeline.encoder.is_null(), "GStreamer pipeline lost its elements");
            for (name, value) in element.settings {
                pipeline.set(name, value);
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

    /// Sets an encoder property from its string form, if the installed version has it.
    fn set(&self, name: &str, value: &str) {
        let name = c_string(name);
        // SAFETY: a GObject's first field is its class pointer
        unsafe {
            let class = *(self.encoder as *const Pointer);
            if (self.api.class_find_property)(class, name.as_ptr()).is_null() {
                log::debug!("{}: no property {name:?}", self.element.name);
                return;
            }
            (self.api.util_set_object_arg)(self.encoder, name.as_ptr(), c_string(value).as_ptr());
        }
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
    /// ready, waiting only once `MAX_IN_FLIGHT` frames are inside (every access unit comes out, in order, one per call).
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
        let in_flight = if self.jpeg.is_some() { jpeg_in_flight() } else { MAX_IN_FLIGHT };
        let timeout = if self.inside.len() as u64 > in_flight { OUTPUT_TIMEOUT } else { Duration::ZERO };
        let data = pull(self.api, self.sink, timeout)?;
        self.check_bus()?;
        Ok(data.map(|data| (data, self.inside.pop_front().unwrap_or(frame_id))))
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
        Ok(GstreamerEncoder { api, element, pipeline: None, last_frame_id: None })
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
        for _ in 0..=MAX_IN_FLIGHT + 1 {
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

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        let DecodedFrame::Video(image) = frame else { bail!("{} takes pictures, got {frame:?}", self.element.name) };
        let size = at_least((target.width, target.height), self.element.min_size);
        // encoders budget each frame from the caps' rate, so a rate that moves far rebuilds the pipeline
        // a JPEG the Jetson's decoder takes goes in as it is (see `Pipeline::new`)
        let jpeg = image.jpeg_bytes().filter(|_| self.element.name == JPEG_ENCODER).and_then(|bytes| Some((bytes, crate::hardware_jpeg::hardware_size(bytes)?)));
        let fresh = self.pipeline.as_ref().is_none_or(|pipeline| pipeline.size != size || pipeline.jpeg != jpeg.map(|(_, jpeg_size)| jpeg_size) || (pipeline.fps / target.fps).max(target.fps / pipeline.fps) > RECONFIGURE_RATIO);
        if fresh {
            self.pipeline = Some(Pipeline::new(self.api, self.element, size, jpeg.map(|(_, jpeg_size)| jpeg_size), target.bitrate_bps, target.fps)?);
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
        Ok(JpegDecoder { api, pipeline: None })
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
            let description = format!("appsrc name=source is-live=true format=time caps=image/jpeg,width={width},height={height},framerate=0/1 ! jpegparse ! nvjpegdec ! nvvidconv ! video/x-raw,format=I420,width={width},height={height} ! appsink name=sink sync=false");
            let (pipeline, source, sink) = launch(self.api, &description)?;
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
            let out = (0..=MAX_IN_FLIGHT + 1).find_map(|_| encoder.encode(&DecodedFrame::Video(image.clone()), &target).unwrap());
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
}
