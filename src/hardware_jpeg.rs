//! JPEG decoding on the Jetson's hardware decoder ([`JpegDecoder`]), in a child process like
//! [`encoders::process`](crate::encoders::process)'s encoder: one that hangs or crashes is killed and started again.
//!
//! The host turns it on with [`enable`] (it starts the children and checks each decodes a test JPEG); from then on the video path
//! ([`compressed_image::to_video`](crate::utils::compressed_image::to_video)) sends each baseline 4:2:0 JPEG with even
//! sides here and gets I420 back, and decodes anything else, or anything the child fails, in software. The child is the
//! host's program again with [`CHILD_ENV`] set: the host calls [`serve_if_child`] first thing in `main`.

use crate::encoders::gstreamer::JpegDecoder;
use crate::encoders::process::ProcessConfig;
use anyhow::{Context, Result, bail, ensure};
use std::io::{BufReader, BufWriter, Read, Write};
use std::process::{Child, ChildStdin, ChildStdout, Stdio};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock};
use std::sync::mpsc;
use std::time::Instant;
use zenoh_gateway::VideoImage;

/// Set in a child started by [`enable`]: `serve_if_child` serves JPEG decodes instead of returning.
pub const CHILD_ENV: &str = "ZENOH_DIMOS_JPEG_DECODER_CHILD";

/// Written by a child once its decoder is made; whatever the program printed to stdout before it is skipped.
const READY: &[u8] = b"\0zenoh-dimos-codecs jpeg decoder ready\0";
/// The most a child may print to stdout before `READY`.
const MAX_BEFORE_READY: usize = 64 * 1024;

/// Children decoding side by side: zenoh-gateway decodes up to 4 frames of a stream ahead at once (its
/// `MAX_DECODE_AHEAD`), and one pipeline's decode is mostly waiting, so they overlap (on the R1, 1920x1536 frames took
/// 31 ms each through one pipeline, 16.5 with two at once, 9.6 with four); with one child they queued behind each other.
const CHILDREN: usize = 4;

/// The children [`enable`] started, each decoding one picture at a time.
static DECODERS: OnceLock<Vec<Mutex<Decoder>>> = OnceLock::new();
/// Set by [`enable`] where the hardware encoder takes JPEG: see [`passthrough`].
static PASSTHROUGH: AtomicBool = AtomicBool::new(false);
/// Where the next decode waits when every child is busy.
static NEXT: AtomicUsize = AtomicUsize::new(0);

/// Turns hardware JPEG decoding on for the video path. Where the Jetson's encoder is installed too, JPEGs go to it as they
/// are ([`passthrough`]) and no decoder child is started: the R1's NVJPG engine opened only ~5 decoders at once, and four
/// idle children left the encoder's own pipeline one. Elsewhere (an Orin Nano has nvjpegdec but no H.264 encoder) it
/// starts the decoder children as `config` says and checks each decodes a test JPEG. Fails (and the video path stays in
/// software) without GStreamer or `nvjpegdec`.
pub fn enable(config: ProcessConfig) -> Result<()> {
    // (here first: without nvjpegdec, no child is started to find out)
    drop(JpegDecoder::hardware()?);
    if crate::encoders::gstreamer::GstreamerEncoder::with_element(crate::encoders::gstreamer::JPEG_ENCODER).is_ok() {
        PASSTHROUGH.store(true, Ordering::Relaxed);
        return Ok(());
    }
    let (jpeg, size) = probe_jpeg()?;
    let mut decoders: Vec<Decoder> = (0..CHILDREN).map(|_| Decoder { config: config.clone(), child: None, starts: 0 }).collect();
    for decoder in &mut decoders {
        decoder.start()?;
    }
    for decoder in &mut decoders {
        let i420 = decoder.decode(&jpeg, size)?;
        ensure!(i420.len() == (size.0 * size.1 * 3 / 2) as usize, "the jpeg decoder process returned {} bytes for {}x{}", i420.len(), size.0, size.1);
    }
    DECODERS.set(decoders.into_iter().map(Mutex::new).collect()).map_err(|_| anyhow::anyhow!("the hardware jpeg decoder is already enabled"))
}

/// Whether [`enable`] turned hardware decoding on.
pub fn enabled() -> bool {
    passthrough() || DECODERS.get().is_some()
}

/// Whether the video path hands JPEGs the hardware decoder takes to the encoder as they are ([`zenoh_gateway::VideoImage::jpeg`]):
/// nvv4l2h264enc's pipeline decodes, scales and encodes them on the GPU, and anything else that reads their pixels
/// decodes them in software then.
pub fn passthrough() -> bool {
    PASSTHROUGH.load(Ordering::Relaxed)
}

/// `data` as I420 from the hardware decoder, if it is enabled and takes this JPEG (baseline, 3 components, 4:2:0, even
/// sides); `Some(Err)` if it failed (the child is restarted for the next). Goes to a child that is free, else waits for one.
pub fn decode(data: &[u8]) -> Option<Result<VideoImage>> {
    let decoders = DECODERS.get()?;
    let size = hardware_size(data)?;
    let mut decoder = decoders.iter().find_map(|decoder| decoder.try_lock().ok()).unwrap_or_else(|| {
        let waited = &decoders[NEXT.fetch_add(1, Ordering::Relaxed) % decoders.len()];
        waited.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    });
    Some(decoder.decode(data, size).and_then(|i420| VideoImage::i420(size.0, size.1, i420)))
}

/// A 64x64 mid-gray 4:2:0 JPEG and its size.
fn probe_jpeg() -> Result<(Vec<u8>, (u32, u32))> {
    let mut jpeg = Vec::new();
    let mut encoder = jpeg_encoder::Encoder::new(&mut jpeg, 90);
    encoder.set_sampling_factor(jpeg_encoder::SamplingFactor::F_2_2);
    encoder.encode(&[128u8; 64 * 64 * 3], 64, 64, jpeg_encoder::ColorType::Rgb).map_err(|error| anyhow::anyhow!("jpeg: {error}"))?;
    Ok((jpeg, (64, 64)))
}

/// The size of a JPEG the hardware decoder takes: baseline (SOF0), 8-bit, three components with luma sampled 2x2 and
/// chroma 1x1 (4:2:0), even sides. Anything else (progressive, 4:2:2, 4:4:4, grayscale, odd sides) is None.
pub fn hardware_size(data: &[u8]) -> Option<(u32, u32)> {
    let mut at = 2;
    while at + 4 <= data.len() {
        if data[at] != 0xff {
            return None;
        }
        let marker = data[at + 1];
        // fill bytes before a marker
        if marker == 0xff {
            at += 1;
            continue;
        }
        let length = u16::from_be_bytes([data[at + 2], data[at + 3]]) as usize;
        match marker {
            0xc0 => {
                let frame = data.get(at + 4..at + 2 + length)?;
                let (precision, height, width, components) = (frame[0], u16::from_be_bytes([frame[1], frame[2]]) as u32, u16::from_be_bytes([frame[3], frame[4]]) as u32, frame[5]);
                let sampling: Vec<u8> = (0..components as usize).filter_map(|component| frame.get(6 + component * 3 + 1).copied()).collect();
                let even = width > 0 && height > 0 && width % 2 == 0 && height % 2 == 0;
                return (precision == 8 && sampling == [0x22, 0x11, 0x11] && even).then_some((width, height));
            }
            // other frame types: progressive, lossless, arithmetic
            0xc1..=0xcf if marker != 0xc4 && marker != 0xc8 && marker != 0xcc => return None,
            // the scan starts before any frame header
            0xda => return None,
            _ => at += 2 + length,
        }
    }
    None
}

/// The parent's side: a child and the thread that talks to it.
struct Decoder {
    config: ProcessConfig,
    child: Option<Running>,
    starts: u64,
}

struct Running {
    child: Child,
    requests: mpsc::Sender<Vec<u8>>,
    replies: mpsc::Receiver<std::result::Result<Reply, String>>,
    started: Instant,
    ready: bool,
}

enum Reply {
    Ready,
    Decoded(std::result::Result<Vec<u8>, String>),
}

impl Decoder {
    fn start(&mut self) -> Result<()> {
        self.starts += 1;
        let mut command = (self.config.command)();
        command.env(CHILD_ENV, "1").stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::inherit());
        let mut child = command.spawn().context("the jpeg decoder process would not start")?;
        let (stdin, stdout) = (child.stdin.take().expect("piped"), child.stdout.take().expect("piped"));
        let (requests, request_rx) = mpsc::channel();
        let (reply_tx, replies) = mpsc::channel();
        std::thread::Builder::new().name("jpeg decoder process".into()).spawn(move || talk(stdin, stdout, request_rx, reply_tx)).expect("a thread");
        self.child = Some(Running { child, requests, replies, started: Instant::now(), ready: false });
        Ok(())
    }

    /// Starts a child if there is none and waits (up to the startup timeout) for it to say it's ready.
    fn wait_ready(&mut self) -> Result<()> {
        if self.child.is_none() {
            self.start()?;
        }
        let running = self.child.as_mut().expect("started");
        if running.ready {
            return Ok(());
        }
        let left = self.config.startup_timeout.saturating_sub(running.started.elapsed());
        let why = match running.replies.recv_timeout(left) {
            Ok(Ok(Reply::Ready)) => {
                running.ready = true;
                return Ok(());
            }
            Ok(Err(error)) => format!("failed to start ({error})"),
            Ok(Ok(Reply::Decoded(_))) => "answered before it was ready".into(),
            Err(mpsc::RecvTimeoutError::Timeout) => format!("did not start within {:?}", self.config.startup_timeout),
            Err(mpsc::RecvTimeoutError::Disconnected) => "exited while starting".into(),
        };
        self.stop(&why);
        bail!("the jpeg decoder process {why}")
    }

    /// Decodes in the child; a child that fails, exits or takes longer than the deadline is killed (the next decode
    /// starts another).
    fn decode(&mut self, data: &[u8], size: (u32, u32)) -> Result<Vec<u8>> {
        self.wait_ready()?;
        let running = self.child.as_mut().expect("ready");
        let mut request = Vec::with_capacity(12 + data.len());
        for value in [size.0, size.1, data.len() as u32] {
            request.extend_from_slice(&value.to_le_bytes());
        }
        request.extend_from_slice(data);
        if running.requests.send(request).is_err() {
            self.stop("exited");
            bail!("the jpeg decoder process exited");
        }
        let why = match running.replies.recv_timeout(self.config.deadline) {
            Ok(Ok(Reply::Decoded(Ok(i420)))) => return Ok(i420),
            Ok(Ok(Reply::Decoded(Err(error)))) => format!("failed ({error})"),
            Ok(Ok(Reply::Ready)) => "said ready twice".into(),
            Ok(Err(error)) => format!("exited ({error})"),
            Err(mpsc::RecvTimeoutError::Timeout) => format!("hung (no picture within {:?})", self.config.deadline),
            Err(mpsc::RecvTimeoutError::Disconnected) => "exited".into(),
        };
        self.stop(&why);
        bail!("the jpeg decoder process {why}: restarted for the next picture")
    }

    /// Closes the child's stdin (which ends a healthy one), kills it (a stuck one) and reaps it off this thread.
    fn stop(&mut self, why: &str) {
        let Some(Running { mut child, requests, .. }) = self.child.take() else { return };
        log::warn!("jpeg decoder process {why}: stopping it (start {})", self.starts);
        drop(requests);
        if let Err(error) = child.kill() {
            log::warn!("jpeg decoder process {}: kill failed: {error}", child.id());
        }
        // a child stuck in the device can take a while to go
        std::thread::spawn(move || {
            if let Err(error) = child.wait() {
                log::warn!("jpeg decoder process {}: wait failed: {error}", child.id());
            }
        });
    }
}

/// The child's I/O, on its own thread: a hung child blocks this thread (until it is killed), never a decode's caller.
fn talk(stdin: ChildStdin, stdout: ChildStdout, requests: mpsc::Receiver<Vec<u8>>, replies: mpsc::Sender<std::result::Result<Reply, String>>) {
    let (mut input, mut output) = (BufWriter::new(stdin), BufReader::new(stdout));
    let mut seen = Vec::new();
    while !seen.ends_with(READY) {
        let mut byte = [0u8];
        if let Err(error) = output.read_exact(&mut byte) {
            let _ = replies.send(Err(format!("no ready signal: {error}")));
            return;
        }
        seen.push(byte[0]);
        if seen.len() > MAX_BEFORE_READY + READY.len() {
            let _ = replies.send(Err(format!("no ready signal in its first {MAX_BEFORE_READY} bytes of output")));
            return;
        }
    }
    if replies.send(Ok(Reply::Ready)).is_err() {
        return;
    }
    for request in requests {
        if let Err(error) = input.write_all(&request).and_then(|()| input.flush()) {
            let _ = replies.send(Err(format!("writing a jpeg: {error}")));
            return;
        }
        let reply = read_reply(&mut output).map_err(|error| format!("{error:#}"));
        let failed = reply.is_err();
        if replies.send(reply.map(Reply::Decoded)).is_err() || failed {
            return;
        }
    }
}

/// In a child [`enable`] started: serves decodes from stdin to stdout and exits (after a decoder error too: the parent
/// starts a fresh child). Anywhere else it returns at once. Call it first thing in `main`.
pub fn serve_if_child() {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    let code = match serve() {
        Ok(()) => 0,
        Err(error) => {
            log::warn!("jpeg decoder process: {error:#}");
            1
        }
    };
    std::process::exit(code);
}

/// Requests: width u32, height u32, length u32, the JPEG; replies: 1, length u32, I420, or 2, length u32, a UTF-8
/// error; little endian.
fn serve() -> Result<()> {
    let mut decoder = JpegDecoder::hardware()?;
    let (mut input, mut output) = (BufReader::new(std::io::stdin().lock()), BufWriter::new(std::io::stdout().lock()));
    output.write_all(READY)?;
    output.flush()?;
    loop {
        let mut head = [0u8; 12];
        match input.read_exact(&mut head) {
            Ok(()) => {}
            // the parent closed the pipe: it is done with this decoder
            Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(error) => return Err(error.into()),
        }
        let field = |index: usize| u32::from_le_bytes(head[index * 4..index * 4 + 4].try_into().expect("4 bytes"));
        let mut jpeg = vec![0u8; field(2) as usize];
        input.read_exact(&mut jpeg)?;
        let reply = decoder.decode(&jpeg, (field(0), field(1))).map_err(|error| format!("{error:#}"));
        match &reply {
            Ok(i420) => {
                output.write_all(&[1])?;
                output.write_all(&(i420.len() as u32).to_le_bytes())?;
                output.write_all(i420)?;
            }
            Err(error) => {
                output.write_all(&[2])?;
                output.write_all(&(error.len() as u32).to_le_bytes())?;
                output.write_all(error.as_bytes())?;
            }
        }
        output.flush()?;
        if let Err(error) = reply {
            bail!("decoder failed: {error}");
        }
    }
}

fn read_reply(input: &mut impl Read) -> Result<std::result::Result<Vec<u8>, String>> {
    let mut head = [0u8; 5];
    input.read_exact(&mut head)?;
    let mut body = vec![0u8; u32::from_le_bytes(head[1..5].try_into().expect("4 bytes")) as usize];
    input.read_exact(&mut body)?;
    Ok(match head[0] {
        1 => Ok(body),
        2 => Err(String::from_utf8_lossy(&body).into_owned()),
        other => bail!("unknown reply {other}"),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_baseline_420_even_jpegs_go_to_hardware() {
        let (jpeg, size) = probe_jpeg().unwrap();
        assert_eq!(hardware_size(&jpeg), Some(size));
        let mut full = Vec::new();
        let mut encoder = jpeg_encoder::Encoder::new(&mut full, 90);
        encoder.set_sampling_factor(jpeg_encoder::SamplingFactor::F_1_1);
        encoder.encode(&[128u8; 64 * 64 * 3], 64, 64, jpeg_encoder::ColorType::Rgb).unwrap();
        assert_eq!(hardware_size(&full), None, "4:4:4");
        let mut odd = Vec::new();
        let mut encoder = jpeg_encoder::Encoder::new(&mut odd, 90);
        encoder.set_sampling_factor(jpeg_encoder::SamplingFactor::F_2_2);
        encoder.encode(&[128u8; 63 * 64 * 3], 63, 64, jpeg_encoder::ColorType::Rgb).unwrap();
        assert_eq!(hardware_size(&odd), None, "odd width");
        assert_eq!(hardware_size(b"\xff\xd8\xff"), None, "truncated");
        assert_eq!(hardware_size(b"not a jpeg"), None);
    }
}
