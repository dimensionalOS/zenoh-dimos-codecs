//! Multiple independent encoder sessions inside one restartable process. Jetson's
//! media driver limits processes, so one child per viewer/camera exhausts it.
use super::*;
use std::collections::VecDeque;
use std::sync::{Mutex, Weak};
use std::sync::atomic::{AtomicU64, Ordering};

/// A bounded pool shared by all live encoders from this factory. Each child goes
/// away with its last session. Busy sessions skip inputs rather than queueing images.
pub fn factory(format: VideoFormat, config: ProcessConfig, processes: usize) -> super::super::Factory {
    let workers: Vec<Mutex<Weak<Mutex<ProcessEncoder>>>> =
        (0..processes.max(1)).map(|_| Mutex::new(Weak::new())).collect();
    let next = AtomicU64::new(1);
    Box::new(move || {
        let id = next.fetch_add(1, Ordering::Relaxed);
        let mut slot = workers[(id as usize - 1) % workers.len()].lock().unwrap_or_else(|e| e.into_inner());
        let worker = slot.upgrade().unwrap_or_else(|| {
            let worker = Arc::new(Mutex::new(ProcessEncoder::new(config.clone(), format)));
            *slot = Arc::downgrade(&worker);
            worker
        });
        Box::new(SharedEncoder {
            worker, id, format,
            generation: 0, last_frame_id: None, pending: None, more: VecDeque::new(),
        })
    })
}

struct SharedEncoder {
    worker: Arc<Mutex<ProcessEncoder>>,
    id: u64,
    format: VideoFormat,
    generation: u64,
    last_frame_id: Option<u64>,
    // Gateway frame IDs increase within a session. Only accepted input counts:
    // a busy worker or a child still starting did not enqueue anything.
    pending: Option<u64>,
    more: VecDeque<(EncodedVideo, Option<u64>)>,
}

impl SharedEncoder {
    fn take_output(&mut self) -> Option<EncodedVideo> {
        let (frame, id) = self.more.pop_front()?;
        self.last_frame_id = id;
        if self.pending.is_some_and(|pending| id.is_none_or(|id| id >= pending)) {
            self.pending = None;
        }
        Some(frame)
    }

    fn exchange(&mut self, request: Vec<u8>, submitted: Option<u64>) -> Result<Option<EncodedVideo>> {
        let Ok(mut worker) = self.worker.try_lock() else { return Ok(None) };
        if worker.child.is_none() { worker.start()?; }
        if self.generation != worker.starts {
            self.more.clear();
            self.last_frame_id = None;
            self.pending = None;
            self.generation = worker.starts;
        }
        if !worker.ready() { return Ok(None); }
        if let Some(id) = submitted { self.pending = Some(id); }
        let first = match worker.exchange(session_request(self.id, request)) {
            Ok(first) => first,
            Err(error) => { self.pending = None; self.more.clear(); return Err(error); }
        };
        if let Some(frame) = first { self.more.push_back((frame, worker.last_frame_id)); }
        self.more.extend(worker.more.drain(..));
        drop(worker);
        Ok(self.take_output())
    }
}

impl VideoEncoder for SharedEncoder {
    fn format(&self) -> VideoFormat { self.format }
    fn last_frame_id(&self) -> Option<u64> { self.last_frame_id }
    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        self.exchange(encode_request(frame, target)?, Some(target.frame_id))
    }
    fn next_output(&mut self, wait: Duration) -> Result<Option<EncodedVideo>> {
        if let Some(frame) = self.take_output() { return Ok(Some(frame)); }
        // Like the in-process pipeline, an empty encoder must return immediately.
        // The gateway drains outputs before releasing completed frames to WebRTC.
        if self.pending.is_none() { return Ok(None); }
        // Wait outside the worker lock: an output finishing between camera inputs
        // must leave immediately, while other sessions can still use the child.
        let due = Instant::now() + wait;
        loop {
            if let Some(frame) = self.exchange(next_request(Duration::ZERO), None)? {
                return Ok(Some(frame));
            }
            if self.pending.is_none() { return Ok(None); }
            let Some(remaining) = due.checked_duration_since(Instant::now()) else { return Ok(None) };
            std::thread::sleep(remaining.min(Duration::from_millis(1)));
        }
    }
}

impl Drop for SharedEncoder {
    fn drop(&mut self) {
        // Serialize the close so it cannot
        // consume another session's reply or leave its hardware pipeline alive.
        let mut worker = self.worker.lock().unwrap_or_else(|e| e.into_inner());
        if worker.child.as_ref().is_some_and(|child| child.ready) {
            let _ = worker.exchange(close_request(self.id));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Pipeline {
        pending: Option<(u64, Instant)>,
        last: Option<u64>,
    }

    impl VideoEncoder for Pipeline {
        fn format(&self) -> VideoFormat { VideoFormat::H264 }
        fn last_frame_id(&self) -> Option<u64> { self.last }
        fn encode(&mut self, _: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
            let delay = if target.frame_id == 1 { Duration::from_millis(20) } else { Duration::ZERO };
            self.pending = Some((target.frame_id, Instant::now() + delay));
            self.next_output(Duration::ZERO)
        }
        fn next_output(&mut self, wait: Duration) -> Result<Option<EncodedVideo>> {
            let Some((id, at)) = self.pending else { return Ok(None) };
            let remaining = at.saturating_duration_since(Instant::now());
            if remaining > wait { return Ok(None); }
            std::thread::sleep(remaining);
            self.pending = None;
            self.last = Some(id);
            Ok(Some(EncodedVideo { data: id.to_le_bytes().to_vec(), width: 16, height: 16, keyframe: true }))
        }
    }

    // Runs in a separate process using the real request/reply protocol, with no camera or GPU.
    #[test]
    #[ignore = "encoder subprocess entry point"]
    fn encoder_child() { serve_if_child(&|| Box::new(Pipeline::default())); }

    fn config() -> ProcessConfig {
        let mut config = ProcessConfig::new(Duration::from_secs(1));
        config.command = Arc::new(|| {
            let mut command = std::process::Command::new(std::env::current_exe().unwrap());
            command.args(["--exact", "encoders::process::shared::tests::encoder_child", "--ignored", "--nocapture"]);
            command
        });
        config
    }

    #[test]
    fn completed_frames_do_not_wait_for_the_next_input_deadline() {
        let factory = factory(VideoFormat::H264, config(), 1);
        let mut encoder = factory();
        let frame = DecodedFrame::Video(VideoImage::i420(16, 16, vec![128; 384]).unwrap());
        let mut target = VideoTarget::new(16, 16, 100_000, 5.0);
        let deadline = Instant::now() + Duration::from_secs(5);
        while encoder.encode(&frame, &target).unwrap().is_none() {
            assert!(Instant::now() < deadline, "encoder child did not start");
            std::thread::sleep(Duration::from_millis(5));
        }
        let start = Instant::now();
        assert!(encoder.next_output(Duration::from_millis(200)).unwrap().is_none());
        assert!(start.elapsed() < Duration::from_millis(50), "an empty encoder held an already completed frame for {:?}", start.elapsed());

        // A genuinely unfinished frame must still be drained between inputs, not lost by returning early.
        target.frame_id = 1;
        assert!(encoder.encode(&frame, &target).unwrap().is_none());
        let start = Instant::now();
        let output = encoder.next_output(Duration::from_millis(200)).unwrap().expect("delayed output");
        assert_eq!(output.data, 1_u64.to_le_bytes());
        assert_eq!(encoder.last_frame_id(), Some(1));
        assert!(start.elapsed() < Duration::from_millis(100));
        let start = Instant::now();
        assert!(encoder.next_output(Duration::from_millis(200)).unwrap().is_none());
        assert!(start.elapsed() < Duration::from_millis(50), "draining the last frame must end the wait");
    }

    #[test]
    fn skipped_input_does_not_create_a_pending_frame() {
        let worker = Arc::new(Mutex::new(ProcessEncoder::new(config(), VideoFormat::H264)));
        let mut encoder = SharedEncoder { worker: worker.clone(), id: 1, format: VideoFormat::H264,
            generation: 0, last_frame_id: None, pending: None, more: VecDeque::new() };
        let frame = DecodedFrame::Video(VideoImage::i420(16, 16, vec![128; 384]).unwrap());
        let target = VideoTarget::new(16, 16, 100_000, 5.0);
        let busy = worker.lock().unwrap();
        assert!(encoder.encode(&frame, &target).unwrap().is_none());
        let start = Instant::now();
        assert!(encoder.next_output(Duration::from_millis(200)).unwrap().is_none());
        assert!(start.elapsed() < Duration::from_millis(50));
        drop(busy);
    }
}
