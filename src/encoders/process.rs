//! A video encoder in a child process, so a hardware encoder that hangs or crashes is killed and started again: the OS
//! frees its threads, memory and device session, which a call stuck inside the gateway's process could never give back
//! (a wedged Jetson encoder, rebuilt in-process every frame, took the R1's Desktop to 640 threads and 3.4 GB).
//!
//! The child is the host's own program again (`std::env::current_exe`, same arguments) with [`CHILD_ENV`] set: the
//! host calls [`serve_if_child`] in `main` as soon as it has its encoder factory, which in the child serves encodes on
//! stdin/stdout and never returns. Frames go over the pipe as raw pictures, the bitstream comes back. In the parent the
//! host registers [`factory`] with zenoh-gateway's `ServerBuilder::video_encoder`, configured by [`ProcessConfig`].

use zenoh_gateway::{DecodedFrame, EncodedVideo, PixelFormat, VideoEncoder, VideoFormat, VideoImage, VideoTarget};
use anyhow::{Context, Result, bail};
use std::io::{BufReader, BufWriter, Read, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// Set in a child started by a [`ProcessEncoder`]: [`serve_if_child`] serves encodes instead of returning.
pub const CHILD_ENV: &str = "ZENOH_DIMOS_VIDEO_ENCODER_CHILD";

/// How a [`ProcessEncoder`] runs its children; the host sets it.
#[derive(Clone)]
pub struct ProcessConfig {
    /// An encode not answered within this restarts the child. Keep it under the gateway's `encode_timeout` (default
    /// 5 s; half of it is a good choice), or the gateway gives up first and the encode keeps a thread.
    pub deadline: Duration,
    /// How long a new child has to make its encoder and say so; frames meanwhile are skipped, not waited on.
    pub startup_timeout: Duration,
    /// Starts a child: the host program again by default ([`ProcessConfig::current_exe`]), which must reach
    /// [`serve_if_child`]; [`CHILD_ENV`] is set on it.
    pub command: Arc<dyn Fn() -> Command + Send + Sync>,
}

impl ProcessConfig {
    /// Children are this program again, with its arguments; `deadline` as above, 20 s to start.
    pub fn new(deadline: Duration) -> Self {
        ProcessConfig { deadline, startup_timeout: Duration::from_secs(20), command: Self::current_exe() }
    }

    /// The host program again, with the arguments it was started with.
    pub fn current_exe() -> Arc<dyn Fn() -> Command + Send + Sync> {
        Arc::new(|| {
            let program = std::env::current_exe().unwrap_or_else(|error| {
                log::warn!("video encoder process: no current_exe ({error}): starting argv[0]");
                std::env::args().next().unwrap_or_default().into()
            });
            let mut command = Command::new(program);
            command.args(std::env::args_os().skip(1));
            command
        })
    }
}

/// A factory for zenoh-gateway's `ServerBuilder::video_encoder`: a [`ProcessEncoder`] of `format` per encode session.
pub fn factory(format: VideoFormat, config: ProcessConfig) -> super::Factory {
    Box::new(move || Box::new(ProcessEncoder::new(config.clone(), format)))
}
/// Written by a child once its encoder is made; whatever the program printed to stdout before it is skipped.
const READY: &[u8] = b"\0zenoh-gateway video encoder ready\0";
/// The most a child may print to stdout before `READY`.
const MAX_BEFORE_READY: usize = 64 * 1024;

/// In a process a [`ProcessEncoder`] started: serves encodes from stdin to stdout with `factory`'s encoder and exits
/// (after an encoder error too: the parent starts a fresh child). Anywhere else it returns at once. Call it in `main`
/// once the encoder factory is known; the child runs `main` again up to here.
pub fn serve_if_child(factory: &(dyn Fn() -> Box<dyn VideoEncoder> + Send + Sync)) {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    let code = match serve(factory) {
        Ok(()) => 0,
        Err(error) => {
            log::warn!("video encoder process: {error:#}");
            1
        }
    };
    std::process::exit(code);
}

fn serve(factory: &(dyn Fn() -> Box<dyn VideoEncoder> + Send + Sync)) -> Result<()> {
    let mut encoder = factory();
    let (mut input, mut output) = (BufReader::new(std::io::stdin().lock()), BufWriter::new(std::io::stdout().lock()));
    output.write_all(READY)?;
    output.flush()?;
    loop {
        // the parent closed the pipe: it is done with this encoder
        let Some((frame, target)) = read_request(&mut input)? else { return Ok(()) };
        let mut reply = encoder.encode(&frame, &target).map(|first| first.map(|frame| (frame, encoder.last_frame_id())).into_iter().collect::<Vec<_>>());
        if let Ok(frames) = &mut reply {
            while let Ok(Some(frame)) = encoder.next_output() {
                frames.push((frame, encoder.last_frame_id()));
            }
        }
        let reply = reply.map_err(|error| format!("{error:#}"));
        write_reply(&mut output, &reply)?;
        output.flush()?;
        if let Err(error) = reply {
            bail!("encoder failed: {error}");
        }
    }
}

/// A [`VideoEncoder`] whose encodes run in a child process (see the module docs): one that takes longer than its
/// deadline, or a child that exits or crashes, kills the child and starts another, whose first frame is a keyframe.
pub struct ProcessEncoder {
    config: ProcessConfig,
    format: VideoFormat,
    child: Option<Running>,
    /// children started for this encoder, the first included
    pub starts: u64,
    /// the child's encoder's `last_frame_id` for the frame last handed out
    last_frame_id: Option<u64>,
    /// frames the last reply had beyond its first, for `next_output`
    more: std::collections::VecDeque<(EncodedVideo, Option<u64>)>,
}

/// A started child and the thread that talks to it.
struct Running {
    child: Child,
    requests: mpsc::Sender<Vec<u8>>,
    replies: mpsc::Receiver<std::result::Result<Reply, String>>,
    started: Instant,
    ready: bool,
}

/// A child's answer to a frame: every frame that came out (none yet, or several from a pipeline catching up), each with
/// whose it is if the encoder's output trails its input.
type Encoded = std::result::Result<Vec<(EncodedVideo, Option<u64>)>, String>;

enum Reply {
    Ready,
    Encoded(Encoded),
}

impl ProcessEncoder {
    /// Encodes `format` in children run as `config` says; the first is started now, so it is ready by the first frames.
    pub fn new(config: ProcessConfig, format: VideoFormat) -> Self {
        let mut encoder = ProcessEncoder { config, format, child: None, starts: 0, last_frame_id: None, more: Default::default() };
        if let Err(error) = encoder.start() {
            log::warn!("{error:#}");
        }
        encoder
    }

    fn start(&mut self) -> Result<()> {
        self.starts += 1;
        let mut command = (self.config.command)();
        command.env(CHILD_ENV, "1").stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::inherit());
        let mut child = command.spawn().context("the video encoder process would not start")?;
        let (stdin, stdout) = (child.stdin.take().expect("piped"), child.stdout.take().expect("piped"));
        let (requests, request_rx) = mpsc::channel();
        let (reply_tx, replies) = mpsc::channel();
        std::thread::Builder::new()
            .name("video encoder process".into())
            .spawn(move || talk(stdin, stdout, request_rx, reply_tx))
            .expect("a thread");
        self.child = Some(Running { child, requests, replies, started: Instant::now(), ready: false });
        Ok(())
    }

    /// Whether the child has said it is ready, without waiting: one that failed, exited or took too long to start is
    /// restarted.
    /// The oldest frame the child sent that isn't handed out yet, its frame id kept for `last_frame_id`.
    fn next_output_now(&mut self) -> Option<EncodedVideo> {
        let (frame, frame_id) = self.more.pop_front()?;
        self.last_frame_id = frame_id;
        Some(frame)
    }

    fn ready(&mut self) -> bool {
        let running = self.child.as_mut().expect("started");
        if running.ready {
            return true;
        }
        let why = match running.replies.try_recv() {
            Ok(Ok(Reply::Ready)) => {
                running.ready = true;
                return true;
            }
            Ok(Err(error)) => format!("failed to start ({error})"),
            Ok(Ok(Reply::Encoded(_))) => "failed to start (answered before it was ready)".into(),
            Err(mpsc::TryRecvError::Empty) if running.started.elapsed() < self.config.startup_timeout => return false,
            Err(mpsc::TryRecvError::Empty) => format!("did not start within {:?}", self.config.startup_timeout),
            Err(mpsc::TryRecvError::Disconnected) => "exited while starting".into(),
        };
        self.restart(&why);
        false
    }

    /// Kills the child (a hung one included: the OS takes back what it held) and starts another.
    fn restart(&mut self, why: &str) {
        log::warn!("video encoder process {why}: restarting it");
        if let Some(running) = self.child.take() {
            running.stop();
        }
        if let Err(error) = self.start() {
            log::warn!("{error:#}");
        }
    }
}

impl Running {
    /// Closes the child's stdin (which ends a healthy child), kills it (a stuck one) and reaps it off this thread.
    fn stop(self) {
        let Running { mut child, requests, .. } = self;
        drop(requests);
        if let Err(error) = child.kill() {
            log::warn!("video encoder process {}: kill failed: {error}", child.id());
        }
        // a child stuck in the device can take a while to go
        std::thread::spawn(move || {
            if let Err(error) = child.wait() {
                log::warn!("video encoder process {}: wait failed: {error}", child.id());
            }
        });
    }
}

impl Drop for ProcessEncoder {
    fn drop(&mut self) {
        if let Some(running) = self.child.take() {
            running.stop();
        }
    }
}

impl VideoEncoder for ProcessEncoder {
    fn format(&self) -> VideoFormat {
        self.format
    }

    fn last_frame_id(&self) -> Option<u64> {
        self.last_frame_id
    }

    fn next_output(&mut self) -> Result<Option<EncodedVideo>> {
        Ok(self.next_output_now())
    }

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        self.last_frame_id = None;
        if self.child.is_none() {
            self.start()?;
        }
        if !self.ready() {
            return Ok(None);
        }
        // a fresh child's first frame is a keyframe, which the viewers need after a restart
        let request = encode_request(frame, target)?;
        let running = self.child.as_mut().expect("started above");
        if running.requests.send(request).is_err() {
            self.restart("exited");
            bail!("the video encoder process exited: restarted");
        }
        match running.replies.recv_timeout(self.config.deadline) {
            Ok(Ok(Reply::Encoded(Ok(frames)))) => {
                self.more.extend(frames);
                Ok(self.next_output_now())
            }
            Ok(Ok(Reply::Encoded(Err(error)))) => {
                self.restart(&format!("failed ({error})"));
                bail!("video encoder: {error}: its process was restarted")
            }
            Ok(Ok(Reply::Ready)) => bail!("the video encoder process said ready twice"),
            Ok(Err(error)) => {
                self.restart(&format!("exited ({error})"));
                bail!("the video encoder process exited ({error}): restarted")
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                self.restart(&format!("hung (no frame within {:?})", self.config.deadline));
                bail!("the video encoder hung for {:?}: its process was restarted", self.config.deadline)
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                self.restart("exited");
                bail!("the video encoder process exited: restarted")
            }
        }
    }
}

/// The child's I/O, on its own thread: a hung child blocks this thread (until it is killed), never the gateway's.
fn talk(stdin: ChildStdin, stdout: ChildStdout, requests: mpsc::Receiver<Vec<u8>>, replies: mpsc::Sender<std::result::Result<Reply, String>>) {
    let (mut input, mut output) = (BufWriter::new(stdin), BufReader::new(stdout));
    let mut seen = Vec::new();
    while !seen.ends_with(READY) {
        let [byte] = match read_array(&mut output) {
            Ok(byte) => byte,
            Err(error) => {
                let _ = replies.send(Err(format!("no ready signal: {error:#}")));
                return;
            }
        };
        seen.push(byte);
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
            let _ = replies.send(Err(format!("writing a frame: {error}")));
            return;
        }
        let reply = read_reply(&mut output).map_err(|error| format!("{error:#}"));
        let failed = reply.is_err();
        if replies.send(reply.map(Reply::Encoded)).is_err() || failed {
            return;
        }
    }
}

/// A request's bytes before the picture's: the layout [`encode_request`] documents.
const REQUEST_HEADER_BYTES: usize = 8 + 4 + 4 + 4 + 8 + 1 + 8 + 1 + 4 + 4 + 4;

/// `target` (quality f64, width u32, height u32, bitrate u32, fps f64, keyframe u8), then the picture (pixel format
/// u8, width u32, height u32, length u32, bytes), little endian.
fn encode_request(frame: &DecodedFrame, target: &VideoTarget) -> Result<Vec<u8>> {
    let DecodedFrame::Video(image) = frame else { bail!("a video encoder process encodes pictures, got {frame:?}") };
    // scaled here, the pipe carries the picture at the size it is encoded at: the R1's full-size 1920x1536 RGB (8.8 MB)
    // piped every frame cost the same at any size, so the gateway's CPU governor shrank it to the floor for nothing
    // a JPEG goes as it is: the child's hardware pipeline decodes and scales it (or the child decodes it, as here)
    let scaled;
    let image = if image.jpeg_bytes().is_some() || (image.width(), image.height()) == (target.width, target.height) {
        image
    } else {
        scaled = image.to_i420(target.width, target.height)?;
        &scaled
    };
    // (a JPEG's data() would decode it: what goes is its bytes)
    let (pixel_format, data) = match image.jpeg_bytes() {
        Some(jpeg) => (2, jpeg),
        None => (match image.format() {
            PixelFormat::Rgb8 => 0,
            PixelFormat::I420 => 1,
        }, image.data()),
    };
    let mut bytes = Vec::with_capacity(REQUEST_HEADER_BYTES + data.len());
    bytes.extend_from_slice(&target.quality.to_le_bytes());
    bytes.extend_from_slice(&target.width.to_le_bytes());
    bytes.extend_from_slice(&target.height.to_le_bytes());
    bytes.extend_from_slice(&target.bitrate_bps.to_le_bytes());
    bytes.extend_from_slice(&target.fps.to_le_bytes());
    bytes.push(target.keyframe as u8);
    bytes.extend_from_slice(&target.frame_id.to_le_bytes());
    bytes.push(pixel_format);
    bytes.extend_from_slice(&image.width().to_le_bytes());
    bytes.extend_from_slice(&image.height().to_le_bytes());
    bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
    bytes.extend_from_slice(data);
    Ok(bytes)
}

fn read_request(input: &mut impl Read) -> Result<Option<(DecodedFrame, VideoTarget)>> {
    let mut head = [0u8; 8];
    match input.read_exact(&mut head) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(error) => return Err(error.into()),
    }
    let quality = f64::from_le_bytes(head);
    let (width, height, bitrate_bps) = (read_u32(input)?, read_u32(input)?, read_u32(input)?);
    let fps = f64::from_le_bytes(read_array(input)?);
    let [keyframe] = read_array(input)?;
    let frame_id = u64::from_le_bytes(read_array(input)?);
    let [pixel_format] = read_array(input)?;
    let (image_width, image_height, length) = (read_u32(input)?, read_u32(input)?, read_u32(input)?);
    let mut data = vec![0u8; length as usize];
    input.read_exact(&mut data).context("a picture's bytes")?;
    let image = match pixel_format {
        0 => VideoImage::rgb8(image_width, image_height, data)?,
        1 => VideoImage::i420(image_width, image_height, data)?,
        2 => VideoImage::jpeg(image_width, image_height, data, crate::utils::compressed_image::decode_jpeg)?,
        other => bail!("unknown pixel format {other}"),
    };
    let mut target = VideoTarget::new(width, height, bitrate_bps, fps);
    (target.quality, target.keyframe, target.frame_id) = (quality, keyframe != 0, frame_id);
    Ok(Some((DecodedFrame::Video(image), target)))
}

/// 1 = frames (count u32, then each: width u32, height u32, keyframe u8, has a frame id u8, frame id u64, length u32,
/// bytes); 2 = an error (length u32, UTF-8), little endian.
fn write_reply(output: &mut impl Write, reply: &Encoded) -> Result<()> {
    match reply {
        Ok(frames) => {
            output.write_all(&[1])?;
            output.write_all(&(frames.len() as u32).to_le_bytes())?;
            for (frame, frame_id) in frames {
                output.write_all(&frame.width.to_le_bytes())?;
                output.write_all(&frame.height.to_le_bytes())?;
                output.write_all(&[frame.keyframe as u8, frame_id.is_some() as u8])?;
                output.write_all(&frame_id.unwrap_or(0).to_le_bytes())?;
                output.write_all(&(frame.data.len() as u32).to_le_bytes())?;
                output.write_all(&frame.data)?;
            }
        }
        Err(error) => {
            output.write_all(&[2])?;
            output.write_all(&(error.len() as u32).to_le_bytes())?;
            output.write_all(error.as_bytes())?;
        }
    }
    Ok(())
}

fn read_reply(input: &mut impl Read) -> Result<Encoded> {
    let [tag] = read_array(input)?;
    Ok(match tag {
        1 => {
            let count = read_u32(input)?;
            let mut frames = Vec::new();
            for _ in 0..count {
                let (width, height) = (read_u32(input)?, read_u32(input)?);
                let [keyframe, has_frame_id] = read_array(input)?;
                let frame_id = u64::from_le_bytes(read_array(input)?);
                let mut data = vec![0u8; read_u32(input)? as usize];
                input.read_exact(&mut data)?;
                frames.push((EncodedVideo { data, width, height, keyframe: keyframe != 0 }, (has_frame_id != 0).then_some(frame_id)));
            }
            Ok(frames)
        }
        2 => {
            let mut text = vec![0u8; read_u32(input)? as usize];
            input.read_exact(&mut text)?;
            Err(String::from_utf8_lossy(&text).into_owned())
        }
        other => bail!("unknown reply {other}"),
    })
}

fn read_array<const N: usize>(input: &mut impl Read) -> Result<[u8; N]> {
    let mut bytes = [0u8; N];
    input.read_exact(&mut bytes)?;
    Ok(bytes)
}

fn read_u32(input: &mut impl Read) -> Result<u32> {
    Ok(u32::from_le_bytes(read_array(input)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_and_replies_round_trip_and_reject_garbage() {
        let image = VideoImage::i420(4, 2, (0..12).collect()).unwrap();
        let mut target = VideoTarget::new(4, 2, 300_000, 15.0);
        (target.quality, target.keyframe, target.frame_id) = (0.25, true, 7);
        let request = encode_request(&DecodedFrame::Video(image), &target).unwrap();
        assert_eq!(request.len(), REQUEST_HEADER_BYTES + 12);
        let (DecodedFrame::Video(decoded), decoded_target) = read_request(&mut request.as_slice()).unwrap().unwrap() else { panic!("a picture") };
        assert_eq!((decoded.width(), decoded.height(), decoded.format(), decoded.data()), (4, 2, PixelFormat::I420, &(0..12).collect::<Vec<u8>>()[..]));
        assert_eq!((decoded_target.quality, decoded_target.bitrate_bps, decoded_target.fps, decoded_target.keyframe, decoded_target.frame_id), (0.25, 300_000, 15.0, true, 7));
        assert!(read_request(&mut &[][..]).unwrap().is_none(), "a closed pipe is the end, not an error");
        // a picture bigger than the target goes over at the target's size
        let big = VideoImage::rgb8(8, 4, vec![200; 8 * 4 * 3]).unwrap();
        let (DecodedFrame::Video(sent), _) = read_request(&mut encode_request(&DecodedFrame::Video(big), &target).unwrap().as_slice()).unwrap().unwrap() else { panic!("a picture") };
        assert_eq!((sent.width(), sent.height(), sent.format()), (4, 2, PixelFormat::I420));
        // a JPEG goes as it is, at its own size, and the child can still decode it
        let jpeg = crate::utils::tests::fixture("compressed/pattern.jpeg");
        let size = crate::utils::compressed_image::decode_jpeg(&jpeg).map(|image| (image.width(), image.height())).unwrap();
        static DECODED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
        let counted = |bytes: &[u8]| {
            DECODED.store(true, std::sync::atomic::Ordering::SeqCst);
            crate::utils::compressed_image::decode_jpeg(bytes)
        };
        let still_jpeg = VideoImage::jpeg(size.0, size.1, jpeg.clone(), counted).unwrap();
        let (DecodedFrame::Video(sent), _) = read_request(&mut encode_request(&DecodedFrame::Video(still_jpeg), &target).unwrap().as_slice()).unwrap().unwrap() else { panic!("a picture") };
        assert_eq!((sent.jpeg_bytes(), sent.width(), sent.height()), (Some(&jpeg[..]), size.0, size.1));
        assert!(!DECODED.load(std::sync::atomic::Ordering::SeqCst), "sending a JPEG never decodes it (it cost 30 ms a frame on the R1)");
        assert!(!sent.data().is_empty());
        assert!(read_request(&mut &request[..request.len() - 1]).is_err(), "a truncated picture");
        let mut unknown_format = request.clone();
        unknown_format[37] = 9;
        assert!(read_request(&mut unknown_format.as_slice()).is_err(), "an unknown pixel format");

        let frame = EncodedVideo { data: vec![0, 0, 0, 1, 0x65], width: 4, height: 2, keyframe: true };
        let later = EncodedVideo { data: vec![0, 0, 0, 1, 0x41], width: 4, height: 2, keyframe: false };
        for reply in [Ok(vec![]), Ok(vec![(frame.clone(), None)]), Ok(vec![(frame, Some(41)), (later, Some(42))]), Err("the device went away".to_owned())] {
            let mut bytes = Vec::new();
            write_reply(&mut bytes, &reply).unwrap();
            assert_eq!(read_reply(&mut bytes.as_slice()).unwrap(), reply);
        }
        assert!(read_reply(&mut &[7u8][..]).is_err(), "an unknown reply tag");
        assert!(read_reply(&mut &[1u8, 1, 0, 0, 0, 4, 0][..]).is_err(), "a truncated frame");
    }
}
