//! macOS VideoToolbox H.264 (the hardware encoder): constrained baseline, low-latency rate control, zenoh-gateway's colors
//! signaled (BT.601 matrix, BT.709 primaries and transfer),
//! one frame in, one access unit out.

use anyhow::{Result, anyhow, bail, ensure};
use std::ffi::c_void;
use std::ptr::{null, null_mut};
use std::sync::Mutex;
use zenoh_gateway::{DecodedFrame, EncodedVideo, VideoEncoder, VideoFormat, VideoTarget};

type CFTypeRef = *const c_void;
type OSStatus = i32;

#[repr(C)]
#[derive(Clone, Copy)]
struct CMTime {
    value: i64,
    timescale: i32,
    flags: u32,
    epoch: i64,
}

type OutputCallback = extern "C" fn(*mut c_void, *mut c_void, OSStatus, u32, CFTypeRef);

#[link(name = "CoreFoundation", kind = "framework")]
unsafe extern "C" {
    static kCFBooleanTrue: CFTypeRef;
    static kCFBooleanFalse: CFTypeRef;
    static kCFTypeDictionaryKeyCallBacks: c_void;
    static kCFTypeDictionaryValueCallBacks: c_void;
    fn CFDictionaryCreate(allocator: CFTypeRef, keys: *const CFTypeRef, values: *const CFTypeRef, count: isize, key_callbacks: *const c_void, value_callbacks: *const c_void) -> CFTypeRef;
    fn CFDictionaryGetValue(dictionary: CFTypeRef, key: CFTypeRef) -> CFTypeRef;
    fn CFNumberCreate(allocator: CFTypeRef, kind: isize, value: *const c_void) -> CFTypeRef;
    fn CFArrayGetCount(array: CFTypeRef) -> isize;
    fn CFArrayGetValueAtIndex(array: CFTypeRef, index: isize) -> CFTypeRef;
    fn CFRelease(object: CFTypeRef);
}

#[link(name = "CoreVideo", kind = "framework")]
unsafe extern "C" {
    static kCVPixelBufferPixelFormatTypeKey: CFTypeRef;
    static kCVPixelBufferWidthKey: CFTypeRef;
    static kCVPixelBufferHeightKey: CFTypeRef;
    static kCVImageBufferColorPrimaries_ITU_R_709_2: CFTypeRef;
    static kCVImageBufferTransferFunction_ITU_R_709_2: CFTypeRef;
    static kCVImageBufferYCbCrMatrix_ITU_R_601_4: CFTypeRef;
    fn CVPixelBufferPoolCreatePixelBuffer(allocator: CFTypeRef, pool: CFTypeRef, out: *mut CFTypeRef) -> i32;
    fn CVPixelBufferLockBaseAddress(buffer: CFTypeRef, flags: u64) -> i32;
    fn CVPixelBufferUnlockBaseAddress(buffer: CFTypeRef, flags: u64) -> i32;
    fn CVPixelBufferGetBaseAddressOfPlane(buffer: CFTypeRef, plane: usize) -> *mut u8;
    fn CVPixelBufferGetBytesPerRowOfPlane(buffer: CFTypeRef, plane: usize) -> usize;
}

#[link(name = "CoreMedia", kind = "framework")]
unsafe extern "C" {
    static kCMTimeInvalid: CMTime;
    static kCMSampleAttachmentKey_NotSync: CFTypeRef;
    fn CMSampleBufferGetDataBuffer(sample: CFTypeRef) -> CFTypeRef;
    fn CMSampleBufferGetFormatDescription(sample: CFTypeRef) -> CFTypeRef;
    fn CMSampleBufferGetSampleAttachmentsArray(sample: CFTypeRef, create: u8) -> CFTypeRef;
    fn CMBlockBufferGetDataLength(buffer: CFTypeRef) -> usize;
    fn CMBlockBufferCopyDataBytes(buffer: CFTypeRef, offset: usize, length: usize, destination: *mut c_void) -> OSStatus;
    fn CMVideoFormatDescriptionGetH264ParameterSetAtIndex(description: CFTypeRef, index: usize, pointer: *mut *const u8, size: *mut usize, count: *mut usize, header_length: *mut i32) -> OSStatus;
}

#[link(name = "VideoToolbox", kind = "framework")]
unsafe extern "C" {
    static kVTVideoEncoderSpecification_EnableLowLatencyRateControl: CFTypeRef;
    static kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder: CFTypeRef;
    static kVTCompressionPropertyKey_RealTime: CFTypeRef;
    static kVTCompressionPropertyKey_ProfileLevel: CFTypeRef;
    static kVTCompressionPropertyKey_AllowFrameReordering: CFTypeRef;
    static kVTCompressionPropertyKey_AverageBitRate: CFTypeRef;
    static kVTCompressionPropertyKey_ExpectedFrameRate: CFTypeRef;
    static kVTCompressionPropertyKey_MaxKeyFrameIntervalDuration: CFTypeRef;
    static kVTCompressionPropertyKey_ColorPrimaries: CFTypeRef;
    static kVTCompressionPropertyKey_TransferFunction: CFTypeRef;
    static kVTCompressionPropertyKey_YCbCrMatrix: CFTypeRef;
    static kVTProfileLevel_H264_ConstrainedBaseline_AutoLevel: CFTypeRef;
    static kVTEncodeFrameOptionKey_ForceKeyFrame: CFTypeRef;
    fn VTCompressionSessionCreate(allocator: CFTypeRef, width: i32, height: i32, codec: u32, specification: CFTypeRef, source_attributes: CFTypeRef, data_allocator: CFTypeRef, callback: OutputCallback, refcon: *mut c_void, out: *mut CFTypeRef) -> OSStatus;
    fn VTSessionSetProperty(session: CFTypeRef, key: CFTypeRef, value: CFTypeRef) -> OSStatus;
    fn VTCompressionSessionPrepareToEncodeFrames(session: CFTypeRef) -> OSStatus;
    fn VTCompressionSessionGetPixelBufferPool(session: CFTypeRef) -> CFTypeRef;
    fn VTCompressionSessionEncodeFrame(session: CFTypeRef, buffer: CFTypeRef, pts: CMTime, duration: CMTime, properties: CFTypeRef, refcon: *mut c_void, flags: *mut u32) -> OSStatus;
    fn VTCompressionSessionCompleteFrames(session: CFTypeRef, until: CMTime) -> OSStatus;
    fn VTCompressionSessionInvalidate(session: CFTypeRef);
}

const CODEC_H264: u32 = u32::from_be_bytes(*b"avc1");
/// NV12, video range: what the encoder takes natively.
const PIXEL_FORMAT_NV12: i32 = i32::from_be_bytes(*b"420v");
const CF_NUMBER_SINT32: isize = 3;
const CF_NUMBER_FLOAT64: isize = 6;
/// kCMTimeFlags_Valid
const CMTIME_FLAGS_VALID: u32 = 1;
/// The lowest frame rate frames are spaced for: zenoh-gateway's own floor (`hz.max(0.1)`).
const MIN_FPS: f64 = 0.1;
/// ExpectedFrameRate is a hint to rate control, so a rate jittering by less than this keeps the session's.
const FPS_CHANGE: f64 = 0.5;

/// An owned CoreFoundation object, released on drop.
struct Owned(CFTypeRef);

impl Drop for Owned {
    fn drop(&mut self) {
        if !self.0.is_null() {
            // SAFETY: we hold one reference
            unsafe { CFRelease(self.0) }
        }
    }
}

fn number_i32(value: i32) -> Owned {
    // SAFETY: CFNumberCreate copies the value
    Owned(unsafe { CFNumberCreate(null(), CF_NUMBER_SINT32, (&raw const value).cast()) })
}

fn number_f64(value: f64) -> Owned {
    // SAFETY: as above
    Owned(unsafe { CFNumberCreate(null(), CF_NUMBER_FLOAT64, (&raw const value).cast()) })
}

fn dictionary(pairs: &[(CFTypeRef, CFTypeRef)]) -> Owned {
    let (keys, values): (Vec<CFTypeRef>, Vec<CFTypeRef>) = pairs.iter().copied().unzip();
    // SAFETY: CF retains the keys and values
    Owned(unsafe { CFDictionaryCreate(null(), keys.as_ptr(), values.as_ptr(), pairs.len() as isize, &raw const kCFTypeDictionaryKeyCallBacks, &raw const kCFTypeDictionaryValueCallBacks) })
}

fn check(status: OSStatus, what: &str) -> Result<()> {
    ensure!(status == 0, "VideoToolbox {what}: OSStatus {status}");
    Ok(())
}

/// The output callback's slot: the access unit of the frame just completed.
type Slot = Mutex<Option<Result<(Vec<u8>, bool)>>>;

extern "C" fn on_output(refcon: *mut c_void, _: *mut c_void, status: OSStatus, _: u32, sample: CFTypeRef) {
    // SAFETY: refcon is the session's boxed slot, alive until the session is invalidated
    let slot = unsafe { &*(refcon as *const Slot) };
    let result = if status != 0 {
        Err(anyhow!("VideoToolbox encode: OSStatus {status}"))
    } else if sample.is_null() {
        Err(anyhow!("VideoToolbox dropped the frame"))
    } else {
        // SAFETY: the sample buffer is valid for the callback's duration
        unsafe { annex_b(sample) }
    };
    *slot.lock().unwrap() = Some(result);
}

/// The sample's access unit as Annex B (4-byte length prefixes become start codes), SPS and PPS first on keyframes.
unsafe fn annex_b(sample: CFTypeRef) -> Result<(Vec<u8>, bool)> {
    unsafe {
        let attachments = CMSampleBufferGetSampleAttachmentsArray(sample, 0);
        let not_sync = if attachments.is_null() || CFArrayGetCount(attachments) == 0 {
            null()
        } else {
            CFDictionaryGetValue(CFArrayGetValueAtIndex(attachments, 0), kCMSampleAttachmentKey_NotSync)
        };
        let keyframe = not_sync.is_null() || not_sync == kCFBooleanFalse;
        let mut out = Vec::new();
        if keyframe {
            let description = CMSampleBufferGetFormatDescription(sample);
            let mut count = 0usize;
            check(CMVideoFormatDescriptionGetH264ParameterSetAtIndex(description, 0, null_mut(), null_mut(), &mut count, null_mut()), "parameter sets")?;
            for index in 0..count {
                let (mut pointer, mut size) = (null(), 0usize);
                check(CMVideoFormatDescriptionGetH264ParameterSetAtIndex(description, index, &mut pointer, &mut size, null_mut(), null_mut()), "parameter set")?;
                out.extend_from_slice(&[0, 0, 0, 1]);
                out.extend_from_slice(std::slice::from_raw_parts(pointer, size));
            }
        }
        let block = CMSampleBufferGetDataBuffer(sample);
        let mut avcc = vec![0u8; CMBlockBufferGetDataLength(block)];
        check(CMBlockBufferCopyDataBytes(block, 0, avcc.len(), avcc.as_mut_ptr().cast()), "copy")?;
        let mut at = 0;
        while at + 4 <= avcc.len() {
            let length = u32::from_be_bytes([avcc[at], avcc[at + 1], avcc[at + 2], avcc[at + 3]]) as usize;
            let nal = avcc.get(at + 4..at + 4 + length).ok_or_else(|| anyhow!("VideoToolbox: truncated NAL unit"))?;
            out.extend_from_slice(&[0, 0, 0, 1]);
            out.extend_from_slice(nal);
            at += 4 + length;
        }
        Ok((out, keyframe))
    }
}

struct Session {
    raw: CFTypeRef,
    slot: Box<Slot>,
    size: (u32, u32),
    bitrate_bps: u32,
    fps: f64,
}

// SAFETY: a compression session may be used from any thread, one call at a time (`&mut self`)
unsafe impl Send for Session {}

impl Drop for Session {
    fn drop(&mut self) {
        // SAFETY: invalidating stops callbacks before the slot is freed
        unsafe {
            VTCompressionSessionInvalidate(self.raw);
            CFRelease(self.raw);
        }
    }
}

impl Session {
    fn new((width, height): (u32, u32), bitrate_bps: u32, fps: f64) -> Result<Self> {
        let slot: Box<Slot> = Box::default();
        // SAFETY: plain CoreFoundation and VideoToolbox calls with valid arguments; `slot` outlives the session
        unsafe {
            let specification = dictionary(&[(kVTVideoEncoderSpecification_EnableLowLatencyRateControl, kCFBooleanTrue), (kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder, kCFBooleanTrue)]);
            let (format, width_number, height_number) = (number_i32(PIXEL_FORMAT_NV12), number_i32(width as i32), number_i32(height as i32));
            let source = dictionary(&[(kCVPixelBufferPixelFormatTypeKey, format.0), (kCVPixelBufferWidthKey, width_number.0), (kCVPixelBufferHeightKey, height_number.0)]);
            let mut raw = null();
            let refcon = (&raw const *slot).cast_mut().cast();
            check(VTCompressionSessionCreate(null(), width as i32, height as i32, CODEC_H264, specification.0, source.0, null(), on_output, refcon, &mut raw), "create")?;
            let session = Session { raw, slot, size: (width, height), bitrate_bps, fps };
            let keyframe_seconds = number_f64(super::KEYFRAME_SECONDS);
            for (key, value) in [
                (kVTCompressionPropertyKey_RealTime, kCFBooleanTrue),
                (kVTCompressionPropertyKey_ProfileLevel, kVTProfileLevel_H264_ConstrainedBaseline_AutoLevel),
                (kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse),
                (kVTCompressionPropertyKey_MaxKeyFrameIntervalDuration, keyframe_seconds.0),
                (kVTCompressionPropertyKey_ColorPrimaries, kCVImageBufferColorPrimaries_ITU_R_709_2),
                (kVTCompressionPropertyKey_TransferFunction, kCVImageBufferTransferFunction_ITU_R_709_2),
                (kVTCompressionPropertyKey_YCbCrMatrix, kCVImageBufferYCbCrMatrix_ITU_R_601_4),
            ] {
                check(VTSessionSetProperty(raw, key, value), "property")?;
            }
            session.set_rate(bitrate_bps, fps)?;
            check(VTCompressionSessionPrepareToEncodeFrames(raw), "prepare")?;
            Ok(session)
        }
    }

    fn set_rate(&self, bitrate_bps: u32, fps: f64) -> Result<()> {
        let (bitrate, rate) = (number_i32(bitrate_bps.min(i32::MAX as u32) as i32), number_f64(fps));
        // SAFETY: valid session and CF values
        unsafe {
            check(VTSessionSetProperty(self.raw, kVTCompressionPropertyKey_AverageBitRate, bitrate.0), "bitrate")?;
            check(VTSessionSetProperty(self.raw, kVTCompressionPropertyKey_ExpectedFrameRate, rate.0), "frame rate")
        }
    }

    /// Copies an I420 picture into a pooled NV12 buffer, encodes it and waits for its access unit.
    fn encode(&mut self, i420: &[u8], pts: CMTime, keyframe: bool) -> Result<(Vec<u8>, bool)> {
        let (width, height) = (self.size.0 as usize, self.size.1 as usize);
        let (luma, chroma) = i420.split_at(width * height);
        let (u, v) = chroma.split_at(width * height / 4);
        // SAFETY: the buffer comes from the session's pool and is locked while written
        unsafe {
            let pool = VTCompressionSessionGetPixelBufferPool(self.raw);
            ensure!(!pool.is_null(), "VideoToolbox: no pixel buffer pool");
            let mut buffer = null();
            check(CVPixelBufferPoolCreatePixelBuffer(null(), pool, &mut buffer), "pixel buffer")?;
            let buffer = Owned(buffer);
            check(CVPixelBufferLockBaseAddress(buffer.0, 0), "lock")?;
            let (y_plane, y_stride) = (CVPixelBufferGetBaseAddressOfPlane(buffer.0, 0), CVPixelBufferGetBytesPerRowOfPlane(buffer.0, 0));
            let (uv_plane, uv_stride) = (CVPixelBufferGetBaseAddressOfPlane(buffer.0, 1), CVPixelBufferGetBytesPerRowOfPlane(buffer.0, 1));
            for (row, source) in luma.chunks_exact(width).enumerate() {
                std::ptr::copy_nonoverlapping(source.as_ptr(), y_plane.add(row * y_stride), width);
            }
            for (row, (u_row, v_row)) in u.chunks_exact(width / 2).zip(v.chunks_exact(width / 2)).enumerate() {
                let destination = std::slice::from_raw_parts_mut(uv_plane.add(row * uv_stride), width);
                for ((pair, u), v) in destination.as_chunks_mut::<2>().0.iter_mut().zip(u_row).zip(v_row) {
                    (pair[0], pair[1]) = (*u, *v);
                }
            }
            check(CVPixelBufferUnlockBaseAddress(buffer.0, 0), "unlock")?;
            let force = keyframe.then(|| dictionary(&[(kVTEncodeFrameOptionKey_ForceKeyFrame, kCFBooleanTrue)]));
            *self.slot.lock().unwrap() = None;
            check(VTCompressionSessionEncodeFrame(self.raw, buffer.0, pts, kCMTimeInvalid, force.as_ref().map_or(null(), |force| force.0), null_mut(), null_mut()), "encode")?;
            check(VTCompressionSessionCompleteFrames(self.raw, pts), "complete")?;
        }
        self.slot.lock().unwrap().take().unwrap_or_else(|| bail!("VideoToolbox returned no frame"))
    }
}

/// H.264 on the Mac's media engine. Re-creates its session when the picture size changes; bitrate and rate changes
/// apply in place.
#[derive(Default)]
pub struct VideoToolboxEncoder {
    session: Option<Session>,
    /// presentation time of the next frame, µs: frames are `1 / fps` apart, so rate control spends `bitrate / fps` on
    /// each (wall-clock stamps gave a burst of frames a fraction of that)
    next_pts: i64,
}

impl VideoEncoder for VideoToolboxEncoder {
    fn format(&self) -> VideoFormat {
        VideoFormat::H264
    }

    fn encode(&mut self, frame: &DecodedFrame, target: &VideoTarget) -> Result<Option<EncodedVideo>> {
        let DecodedFrame::Video(image) = frame else { bail!("VideoToolbox takes pictures, got {frame:?}") };
        let size = (target.width, target.height);
        let fresh = self.session.as_ref().is_none_or(|session| session.size != size);
        if fresh {
            self.session = Some(Session::new(size, target.bitrate_bps, target.fps)?);
        }
        let session = self.session.as_mut().expect("created above");
        if session.bitrate_bps != target.bitrate_bps || (session.fps - target.fps).abs() > FPS_CHANGE {
            session.set_rate(target.bitrate_bps, target.fps)?;
            (session.bitrate_bps, session.fps) = (target.bitrate_bps, target.fps);
        }
        let i420 = image.to_i420(target.width, target.height)?;
        let pts = CMTime { value: self.next_pts, timescale: 1_000_000, flags: CMTIME_FLAGS_VALID, epoch: 0 };
        self.next_pts += (1e6 / target.fps.max(MIN_FPS)).round().max(1.0) as i64;
        let (data, keyframe) = session.encode(i420.data(), pts, target.keyframe || fresh)?;
        Ok(Some(EncodedVideo { data, width: target.width, height: target.height, keyframe }))
    }
}
