//! How fast the gateway's own H.264 encoder is at the camera's size, which decides whether the re-encode path needs a
//! hardware encoder as well as a hardware decoder.
//!
//! Run it where the answer matters — the robot, not a developer's laptop:
//! `cargo test --release --test encode_throughput -- --nocapture`
//!
//! It encodes 720p pictures with [`H264Encoder`], the encoder a gateway uses unless a hardware one is plugged in, and
//! prints frames per second. Two cameras at 30 Hz need 60 fps of encode on top of their decode, so under about 80 fps
//! here means the encode half needs hardware too (RK3588: v4l2 h264_v4l2m2m, or MPP directly).
//!
//! The pictures are real decoded video where libde265 is installed, because an encoder's speed depends on its content:
//! a still or synthetic picture compresses to almost nothing and flatters it.

use std::time::Instant;
use zenoh_gateway::{DecodedFrame, VideoEncoder, VideoImage, VideoTarget};
use zenoh_gateway::H264Encoder;
use zenoh_dimos_codecs::utils::compressed_video::{Codec, nal_units};
use zenoh_dimos_codecs::utils::h265;

fn fixture(name: &str) -> Vec<u8> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("test/fixtures").join(name);
    std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}

/// The robot's capture, decoded: real 1280x720 pictures for the encoder to chew on.
fn pictures() -> Vec<VideoImage> {
    let Ok(mut stream) = h265::Stream::new() else {
        eprintln!("libde265 is not installed: no real pictures, so nothing to measure honestly");
        return Vec::new();
    };
    let keyframe = fixture("compressed/m20_front_keyframe.h265");
    let mut sets = Vec::new();
    for (kind, unit) in nal_units(Codec::H265, &keyframe) {
        if (32..=34).contains(&kind) {
            sets.extend_from_slice(&[0, 0, 0, 1]);
            sets.extend_from_slice(unit);
        }
    }
    let mut pictures = Vec::new();
    if let Ok(Some(first)) = stream.push(&keyframe, true) {
        pictures.push(first);
    }
    let capture = fixture("compressed/m20_front_2s.h265");
    for (kind, unit) in nal_units(Codec::H265, &capture) {
        let keyframe = (16..=21).contains(&kind);
        let mut access_unit = if keyframe { sets.clone() } else { Vec::new() };
        access_unit.extend_from_slice(&[0, 0, 0, 1]);
        access_unit.extend_from_slice(unit);
        if let Ok(Some(picture)) = stream.push(&access_unit, keyframe) {
            pictures.push(picture);
        }
    }
    pictures
}

#[test]
fn h264_encode_throughput_at_the_camera_s_size() {
    let pictures = pictures();
    if pictures.is_empty() {
        return;
    }
    let (width, height) = (pictures[0].width(), pictures[0].height());
    let mut encoder = H264Encoder::default();
    // 4 Mbit/s at 30 Hz, which is the sort of grant a camera on a local link gets
    let target = VideoTarget::new(width, height, 4_000_000, 30.0);
    // one pass to build the encoder and settle its rate control, then the measured pass
    for picture in &pictures {
        let _ = encoder.encode(&DecodedFrame::Video(picture.clone()), &target);
    }
    let started = Instant::now();
    let mut encoded = 0;
    let mut bytes = 0usize;
    for picture in &pictures {
        if let Ok(Some(frame)) = encoder.encode(&DecodedFrame::Video(picture.clone()), &target) {
            encoded += 1;
            bytes += frame.data.len();
        }
    }
    let elapsed = started.elapsed().as_secs_f64();
    let fps = encoded as f64 / elapsed;
    println!(
        "openh264: {encoded} frames of {width}x{height} in {:.0} ms = {fps:.1} fps ({:.1} ms/frame, {:.2} Mbit/s at 30 Hz)",
        elapsed * 1000.0,
        elapsed * 1000.0 / encoded.max(1) as f64,
        bytes as f64 * 8.0 / encoded.max(1) as f64 * 30.0 / 1e6,
    );
    println!("two cameras at 30 Hz need 60 fps of encode: this machine {}", if fps >= 80.0 { "has room" } else { "does NOT — the encode half needs hardware too" });
    assert!(encoded > 20, "only {encoded} frames encoded");
}

/// `ZDC_VIDEO_ENCODER` is the way past openh264 on a machine with no hardware encoder, which is what an RK3588 is.
/// This says the selection works and prints what the named element does against the software default.
#[test]
#[cfg(feature = "gstreamer")]
fn a_named_gstreamer_encoder_is_selected_and_measured() {
    use zenoh_dimos_codecs::encoders::{Backend, select};
    // SAFETY: single-threaded test; the variable is read only inside `select`
    unsafe { std::env::remove_var("ZDC_VIDEO_ENCODER") };
    let plain = select(Backend::Auto).expect("auto always selects something");
    println!("unset:                 {}", plain.name);

    // SAFETY: as above
    unsafe { std::env::set_var("ZDC_VIDEO_ENCODER", "x264enc") };
    let named = select(Backend::Auto).expect("auto falls back to software if x264enc is missing");
    println!("ZDC_VIDEO_ENCODER=x264enc: {}", named.name);
    // SAFETY: as above
    unsafe { std::env::remove_var("ZDC_VIDEO_ENCODER") };

    let Some(factory) = named.factory else {
        println!("x264enc is not installed here: nothing to measure");
        return;
    };
    assert_eq!(named.name, "gstreamer x264enc", "the named element was selected");

    let pictures = pictures();
    if pictures.is_empty() {
        return;
    }
    let (width, height) = (pictures[0].width(), pictures[0].height());
    let target = VideoTarget::new(width, height, 4_000_000, 30.0);
    let mut encoder = factory();
    for picture in &pictures {
        let _ = encoder.encode(&DecodedFrame::Video(picture.clone()), &target);
    }
    let started = Instant::now();
    let mut encoded = 0;
    for picture in &pictures {
        if let Ok(Some(_)) = encoder.encode(&DecodedFrame::Video(picture.clone()), &target) {
            encoded += 1;
        }
    }
    let elapsed = started.elapsed().as_secs_f64();
    println!("x264enc: {encoded} frames of {width}x{height} in {:.0} ms = {:.1} fps ({:.1} ms/frame)", elapsed * 1000.0, encoded as f64 / elapsed, elapsed * 1000.0 / encoded.max(1) as f64);
    assert!(encoded > 20, "only {encoded} frames encoded");
}

/// How much latency the GStreamer encoder holds, which is a different question from how fast it is.
///
/// `GstreamerEncoder::encode` pushes a frame and only *waits* for output once `MAX_IN_FLIGHT` frames are inside;
/// before that it takes whatever is already out, which for a 1-in-1-out encoder is nothing. That exists for hardware
/// encoders that take several frame times per frame. For a software encoder in zerolatency mode it is pure added
/// delay — and worse, the gateway measures the wall time of `encode` as the encoder's cost, so queueing shows up as
/// encode time and the CPU governor shrinks the picture in response.
///
/// So: feed frames at the rate a camera sends them and report how many frames the encoder is holding and how long
/// each call takes. Standing depth near `MAX_IN_FLIGHT` means ~that many frame intervals of latency by construction.
#[test]
#[cfg(feature = "gstreamer")]
fn how_many_frames_the_gstreamer_encoder_holds() {
    use zenoh_dimos_codecs::encoders::{Backend, select};
    // SAFETY: single-threaded test
    unsafe { std::env::set_var("ZDC_VIDEO_ENCODER", "x264enc") };
    let selected = select(Backend::Auto).expect("auto selects something");
    // SAFETY: as above
    unsafe { std::env::remove_var("ZDC_VIDEO_ENCODER") };
    let Some(factory) = selected.factory.filter(|_| selected.name.contains("x264enc")) else {
        println!("x264enc not selected here ({}): nothing to measure", selected.name);
        return;
    };

    let pictures = pictures();
    if pictures.is_empty() {
        return;
    }
    let (width, height) = (pictures[0].width(), pictures[0].height());
    let target = VideoTarget::new(width, height, 4_000_000, 30.0);
    let mut encoder = factory();
    let frame_interval = std::time::Duration::from_millis(33);

    let (mut out, mut none_before_first, mut first_seen) = (0usize, 0usize, false);
    let mut call_ms: Vec<f64> = Vec::new();
    for round in 0..90 {
        let picture = &pictures[round % pictures.len()];
        let at = Instant::now();
        let got = encoder.encode(&DecodedFrame::Video(picture.clone()), &target).expect("encode");
        call_ms.push(at.elapsed().as_secs_f64() * 1000.0);
        match got {
            Some(_) => {
                out += 1;
                first_seen = true;
            }
            None if !first_seen => none_before_first += 1,
            None => {}
        }
        // a camera's pace, so the measurement is about holding rather than throughput
        std::thread::sleep(frame_interval.saturating_sub(at.elapsed()));
    }
    call_ms.sort_by(f64::total_cmp);
    let p50 = call_ms[call_ms.len() / 2];
    let max = call_ms[call_ms.len() - 1];
    println!("x264enc at {width}x{height}, fed at 30 Hz for 90 frames:");
    println!("  frames out: {out} of 90   held back before the first came out: {none_before_first}");
    println!("  encode() call: p50 {p50:.1} ms, max {max:.1} ms");
    println!("  => the gateway reads that call time as the encoder's cost; its governor shrinks the picture past ~28 ms");
    assert!(out > 60, "only {out} of 90 frames came out");
}
