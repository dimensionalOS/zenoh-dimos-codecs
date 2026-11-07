//! The whole `video-h265` path, in one process, over a real WebRTC connection: a camera's own bitstream published on
//! zenoh, through the compressed-video encoding and the gateway's track, back out of the Rust client.
//!
//! The parts are unit-tested on their own, which says nothing about whether they fit together: the codec has to put the
//! access unit where the passthrough encoder looks for it, the gateway has to negotiate `video/H265` with the client,
//! the payloader has to packetize a real 37 KB keyframe into RTP, and the depacketizer has to rebuild it. This test is
//! the only thing that exercises all of that, and it is what says the keyframes-only limit is actually gone — the
//! assertion that matters is that **predicted frames arrive**, because the old path could only ever deliver one frame
//! per keyframe.
//!
//! `cargo test --test h265_passthrough` (needs the gateway's `client` feature, which dev-dependencies turn on).

use anyhow::Result;
use std::time::{Duration, Instant};
use zenoh_dimos_codecs::codecs::msgs::dimos_lcm_compressed_video::DimosLcmCompressedVideo;
use zenoh_dimos_codecs::codecs::msgs::ros2_compressed_video::Ros2CompressedVideo;
use zenoh_gateway::client::{Client, ClientOptions, Message, SubscribeOptions};
use zenoh_gateway::{RunningServer, Server, zenoh};

/// `foxglove_msgs/CompressedVideo` as rmw_zenoh puts it on the wire: little-endian CDR, a `std_msgs/Header`
/// (`builtin_interfaces/Time` then `frame_id`), the access unit as a byte sequence, then the format's name.
fn compressed_video_cdr(data: &[u8], format: &str) -> Vec<u8> {
    // the encapsulation header is not part of the body, and every alignment below is counted from the body's start
    let mut body: Vec<u8> = Vec::with_capacity(data.len() + 64);
    let align = |body: &mut Vec<u8>, to: usize| while body.len() % to != 0 { body.push(0) };
    let string = |body: &mut Vec<u8>, text: &str| {
        align(body, 4);
        body.extend_from_slice(&(text.len() as u32 + 1).to_le_bytes());
        body.extend_from_slice(text.as_bytes());
        body.push(0);
    };
    body.extend_from_slice(&1_700_000_000i32.to_le_bytes());
    body.extend_from_slice(&123_456_789u32.to_le_bytes());
    string(&mut body, "camera_color_optical_frame");
    align(&mut body, 4);
    body.extend_from_slice(&(data.len() as u32).to_le_bytes());
    body.extend_from_slice(data);
    string(&mut body, format);
    [&[0x00, 0x01, 0x00, 0x00][..], &body].concat()
}

/// `foxglove_msgs.CompressedVideo` as dimos puts it on the wire: big-endian LCM behind its fingerprint — the data's
/// length, the timestamp, the frame id, the data, then the format's name. This is the encoding the **robot** publishes;
/// the CDR one above is what a ROS 2 system would.
fn compressed_video_lcm(data: &[u8], format: &str) -> Vec<u8> {
    /// the fingerprint `lcm-gen` computes for dimos-lcm's type, as the codec checks for
    const FINGERPRINT: [u8; 8] = [0x22, 0x17, 0xb4, 0x6e, 0xef, 0xbf, 0x7b, 0x05];
    let mut out = FINGERPRINT.to_vec();
    let string = |out: &mut Vec<u8>, text: &str| {
        out.extend_from_slice(&(text.len() as i32 + 1).to_be_bytes());
        out.extend_from_slice(text.as_bytes());
        out.push(0);
    };
    out.extend_from_slice(&(data.len() as i32).to_be_bytes());
    out.extend_from_slice(&1_700_000_000i32.to_be_bytes());
    out.extend_from_slice(&0i32.to_be_bytes());
    string(&mut out, "camera_color_optical_frame");
    out.extend_from_slice(data);
    string(&mut out, format);
    out
}

fn fixture(name: &str) -> Vec<u8> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("test/fixtures").join(name);
    std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}

/// The NAL units of an Annex-B buffer, each with a four-byte start code, and whether it is an IRAP picture.
fn access_units(data: &[u8]) -> Vec<(bool, Vec<u8>)> {
    let mut units = Vec::new();
    let mut starts = Vec::new();
    let mut at = 0;
    while let Some(found) = data.get(at..).and_then(|rest| rest.windows(3).position(|window| window == [0, 0, 1])) {
        at += found + 3;
        starts.push(at);
    }
    for (index, &start) in starts.iter().enumerate() {
        let end = starts.get(index + 1).map_or(data.len(), |&next| next - 3);
        let unit = &data[start..end];
        let unit = &unit[..unit.iter().rposition(|&byte| byte != 0).map_or(0, |last| last + 1)];
        if let Some(&first) = unit.first() {
            let kind = (first >> 1) & 0x3f;
            // the parameter sets ride with the keyframe that follows them, as the dimos relay publishes them
            if (32..=34).contains(&kind) {
                continue;
            }
            units.push(((16..=21).contains(&kind), [&[0, 0, 0, 1][..], unit].concat()));
        }
    }
    units
}

fn isolated_config() -> zenoh::Config {
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false").unwrap();
    config.insert_json5("listen/endpoints", "[]").unwrap();
    config
}

async fn start() -> (RunningServer, zenoh::Session, String) {
    let session = zenoh::open(isolated_config()).await.unwrap();
    let server = Server::builder().session(session.clone()).encoding(Ros2CompressedVideo).encoding(DimosLcmCompressedVideo).build().await.unwrap();
    let running = server.bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", running.local_addr());
    (running, session, url)
}

/// Publishes the robot's capture on `key` at 30 Hz until dropped: the keyframe with its parameter sets, then the
/// predicted frames that follow it, over and over.
fn keep_publishing(session: &zenoh::Session, key: &str) -> tokio::task::JoinHandle<()> {
    keep_publishing_as(session, key, compressed_video_cdr)
}

/// As [`keep_publishing`], in whichever wire format `encode` writes.
fn keep_publishing_as(session: &zenoh::Session, key: &str, encode: fn(&[u8], &str) -> Vec<u8>) -> tokio::task::JoinHandle<()> {
    let keyframe = fixture("compressed/m20_front_keyframe.h265");
    let predicted: Vec<Vec<u8>> = access_units(&fixture("compressed/m20_front_2s.h265")).into_iter().filter(|(keyframe, _)| !keyframe).map(|(_, unit)| unit).collect();
    assert!(predicted.len() > 20, "the capture has {} predicted frames", predicted.len());
    let (session, key) = (session.clone(), key.to_owned());
    tokio::spawn(async move {
        let frames: Vec<Vec<u8>> = [vec![keyframe], predicted].concat();
        for access_unit in frames.iter().cycle() {
            if session.put(&key, encode(access_unit, "h265")).await.is_err() {
                return;
            }
            tokio::time::sleep(Duration::from_millis(33)).await;
        }
    })
}

/// The other half: a client with no HEVC decoder (Firefox) asks for `video-h264`, so the gateway decodes the stream
/// with its stateful decoder and encodes it again. The per-sample decode could only manage keyframes, about one a
/// second, so again the assertion that matters is that far more than that arrives.
#[tokio::test(flavor = "multi_thread")]
async fn a_client_without_hevc_gets_every_frame_re_encoded() -> Result<()> {
    if zenoh_dimos_codecs::utils::h265::Stream::new().is_err() {
        eprintln!("skipping: libde265 is not installed");
        return Ok(());
    }
    let (running, session, url) = start().await;
    let _publisher = keep_publishing(&session, "camera/front");
    let client = Client::connect(&url, ClientOptions::default()).await?;
    let options = SubscribeOptions { encoding: Some("ros2_compressed_video".to_owned()), channel: Some("video-h264".to_owned()), metadata: Some(true), ..Default::default() };
    let mut camera = client.subscribe("camera/front", options).await?;

    let (mut frames, mut info) = (0usize, Vec::new());
    let started = Instant::now();
    while frames < 20 && started.elapsed() < Duration::from_secs(25) {
        match tokio::time::timeout(Duration::from_secs(12), camera.recv()).await {
            Ok(Some(Message::Video(frame))) => {
                assert_eq!(frame.format, zenoh_gateway::VideoFormat::H264, "re-encoded to H.264 for a browser with no HEVC");
                frames += 1;
            }
            Ok(Some(Message::VideoInfo(frame))) => info.push(frame),
            Ok(Some(_)) => {}
            Ok(None) | Err(_) => break,
        }
    }
    let elapsed = started.elapsed();
    // the publisher sends one keyframe per 30 frames at 30 Hz, so a keyframes-only decode would manage about one a
    // second: 20 frames in well under 20 s can only mean predicted frames were decoded
    assert!(frames >= 20, "only {frames} re-encoded frames in {elapsed:?}");
    assert!(elapsed < Duration::from_secs(15), "{frames} frames took {elapsed:?}: that is keyframe pace, so the stateful decode is not working");
    // the pictures are the camera's own size, decoded rather than read off an SPS
    let sized = info.iter().find(|frame| frame.source_width > 0).expect("metadata with a source size");
    assert_eq!((sized.source_width, sized.source_height), (1280, 720));
    // Resyncs, but not a storm of them. A machine that cannot encode this picture at this rate *should* drop samples
    // and resync - that is the stream degrading honestly, not a defect, and a debug build on a developer machine does
    // exactly that once the picture is full size. What would be a defect is resyncing faster than frames arrive, which
    // is what the read-ahead bug did. (This asserted zero until the grant ratchet was fixed, and passed only because
    // the ratchet had shrunk the picture to something a slow build could keep up with - green for the wrong reason.)
    let stats = client.stats().await?;
    let resyncs = stats["channels"].as_array().and_then(|channels| channels.first()).map(|channel| channel["stats"]["streamResyncs"].as_u64().unwrap_or(0)).unwrap_or(0);
    assert!(resyncs as usize * 5 < frames, "the decoder started over {resyncs} times for {frames} frames; stats: {}", stats["channels"]);

    drop(camera);
    client.close().await;
    running.shutdown().await?;
    Ok(())
}

/// Predicted frames reach the browser, which is the whole point: the keyframes-only decode could deliver one frame per
/// keyframe, so anything over that is a frame that used to be lost.
#[tokio::test(flavor = "multi_thread")]
async fn predicted_frames_reach_a_client_that_can_decode_h265() -> Result<()> {
    let (running, session, url) = start().await;
    let _publisher = keep_publishing(&session, "camera/front");
    let client = Client::connect(&url, ClientOptions::default()).await?;
    let options = SubscribeOptions { encoding: Some("ros2_compressed_video".to_owned()), channel: Some("video-h265".to_owned()), metadata: Some(true), ..Default::default() };
    let mut camera = client.subscribe("camera/front", options).await?;

    let (mut frames, mut keyframes, mut info) = (Vec::new(), 0, Vec::new());
    let started = Instant::now();
    while frames.len() < 20 && started.elapsed() < Duration::from_secs(20) {
        match tokio::time::timeout(Duration::from_secs(10), camera.recv()).await {
            Ok(Some(Message::Video(frame))) => {
                keyframes += usize::from(frame.keyframe);
                frames.push(frame);
            }
            Ok(Some(Message::VideoInfo(frame))) => info.push(frame),
            Ok(Some(_)) => {}
            Ok(None) => break,
            Err(_) => break,
        }
    }

    assert!(frames.len() >= 20, "only {} frames in {:?}", frames.len(), started.elapsed());
    assert!(keyframes >= 1, "no keyframe arrived, so nothing could have been decoded");
    assert!(frames.len() > keyframes, "{} of {} frames were keyframes: predicted frames are not getting through", keyframes, frames.len());
    // the negotiated format is H.265, not something the gateway re-encoded to
    assert!(frames.iter().all(|frame| frame.format == zenoh_gateway::VideoFormat::H265), "{:?}", frames[0].format);
    // and what came back is an Annex-B access unit, which is what a browser's decoder is handed
    assert!(frames.iter().all(|frame| frame.data.windows(4).any(|window| window == [0, 0, 0, 1])), "no Annex-B start code in a frame");
    // the first frame a viewer can use carries the parameter sets, so it decodes on its own
    let first_key = frames.iter().find(|frame| frame.keyframe).expect("a keyframe");
    let types: Vec<u8> = {
        let mut types = Vec::new();
        let mut at = 0;
        while let Some(found) = first_key.data.get(at..).and_then(|rest| rest.windows(3).position(|window| window == [0, 0, 1])) {
            at += found + 3;
            if let Some(&first) = first_key.data.get(at) {
                types.push((first >> 1) & 0x3f);
            }
        }
        types
    };
    assert!(types.contains(&32) && types.contains(&33) && types.contains(&34), "the keyframe has no VPS/SPS/PPS (types {types:?})");
    assert!(types.iter().any(|kind| (16..=21).contains(kind)), "the keyframe has no IRAP picture (types {types:?})");
    // the size the gateway reports comes from the sequence parameter set, since nothing decoded the picture
    let sized = info.iter().find(|frame| frame.width > 0).expect("metadata with a size");
    assert_eq!((sized.width, sized.height), (1280, 720), "the gateway read the size off the SPS");

    drop(camera);
    client.close().await;
    running.shutdown().await?;
    Ok(())
}

/// The deployment this whole thing exists for: a Chrome and a Firefox watching **one** camera at the same time, one
/// passing the H.265 through and one having it decoded and re-encoded.
///
/// What it checks is that the two paths coexist: two connections, two subscriptions and two very different decoders
/// against one publisher, each getting the format it negotiated, at a rate neither starves, both reporting the same
/// source picture. Nothing before this ran them together.
///
/// What it does **not** check, though it was written to: the decode cache's key. That key was
/// `(encoding, channel.kind(), hash)`, and `kind()` is `EncodingOutput::Video` for `video-h265` and `video-h264`
/// alike, so two video subscriptions could be handed each other's frames. Keying it on the whole `Channel` fixed it —
/// but this test passes with that fix reverted, because the two paths no longer meet in the cache at all: a
/// subscription with a `StreamDecoder` decodes its own samples and never calls `decode_shared`, so only the
/// passthrough side uses the cache. The key is still right (it is what actually varies) and still protects an encoding
/// that decodes per channel without a stream decoder; it is simply no longer load-bearing for this pair, and this test
/// is no evidence either way.
#[tokio::test(flavor = "multi_thread")]
async fn both_browsers_watch_one_camera_at_once() -> Result<()> {
    if zenoh_dimos_codecs::utils::h265::Stream::new().is_err() {
        eprintln!("skipping: libde265 is not installed");
        return Ok(());
    }
    let (running, session, url) = start().await;
    let _publisher = keep_publishing(&session, "camera/front");

    // two clients, as two browsers are: separate connections, separate subscriptions, one publisher
    let chrome = Client::connect(&url, ClientOptions::default()).await?;
    let firefox = Client::connect(&url, ClientOptions::default()).await?;
    let options = |channel: &str| SubscribeOptions {
        encoding: Some("ros2_compressed_video".to_owned()),
        channel: Some(channel.to_owned()),
        metadata: Some(true),
        ..Default::default()
    };
    let mut passthrough = chrome.subscribe("camera/front", options("video-h265")).await?;
    let mut reencoded = firefox.subscribe("camera/front", options("video-h264")).await?;

    // both are read at once, so neither can be starved by the other being drained first
    let ((h265_frames, h265_sizes, h265_time), (h264_frames, h264_sizes, h264_time)) = tokio::join!(
        take_frames(&mut passthrough, zenoh_gateway::VideoFormat::H265),
        take_frames(&mut reencoded, zenoh_gateway::VideoFormat::H264),
    );

    // both well past the one-keyframe-a-second the old path managed, and neither starved by the other
    assert!(h265_frames >= 15, "passthrough got {h265_frames} frames in {h265_time:?} while the other ran");
    assert!(h264_frames >= 15, "re-encode got {h264_frames} frames in {h264_time:?} while the other ran");
    // the same camera, so both report the same source picture - if the cache were crossed these would disagree
    assert!(h265_sizes.iter().all(|size| *size == (1280, 720)), "passthrough source sizes {h265_sizes:?}");
    assert!(h264_sizes.iter().all(|size| *size == (1280, 720)), "re-encode source sizes {h264_sizes:?}");

    drop(passthrough);
    drop(reencoded);
    chrome.close().await;
    firefox.close().await;
    running.shutdown().await?;
    Ok(())
}

/// Reads frames off `subscription` until it has 15 or runs out of patience, checking every one is `want`; gives back
/// how many came, the source sizes their metadata reported, and how long it took.
async fn take_frames(
    subscription: &mut zenoh_gateway::client::Subscription,
    want: zenoh_gateway::VideoFormat,
) -> (usize, Vec<(u32, u32)>, Duration) {
    let (mut frames, mut sizes) = (0usize, Vec::new());
    let started = Instant::now();
    while frames < 15 && started.elapsed() < Duration::from_secs(25) {
        match tokio::time::timeout(Duration::from_secs(12), subscription.recv()).await {
            Ok(Some(Message::Video(frame))) => {
                assert_eq!(frame.format, want, "a {want:?} subscription got {:?}", frame.format);
                frames += 1;
            }
            Ok(Some(Message::VideoInfo(info))) if info.source_width > 0 => sizes.push((info.source_width, info.source_height)),
            Ok(Some(_)) => {}
            Ok(None) | Err(_) => break,
        }
    }
    (frames, sizes, started.elapsed())
}

/// The LCM builder above must produce what the real parser reads, or a test built on it proves nothing.
#[test]
fn the_lcm_builder_round_trips_through_the_codec_s_own_parser() {
    use zenoh_dimos_codecs::codecs::msgs::dimos_lcm_compressed_video::parse;
    let data = fixture("compressed/m20_front_keyframe.h265");
    let payload = compressed_video_lcm(&data, "h265");
    let message = parse(&payload).expect("the codec parses what we built");
    assert_eq!(message.format.as_str(), "h265");
    assert_eq!(message.data, data.as_slice());
    assert_eq!(message.header.frame_id, "camera_color_optical_frame");
}

/// The robot's own wire format through the re-encode path, which nothing covered: every end-to-end test above drives
/// `ros2_compressed_video` (CDR), while the M20's camera publishes `dimos_lcm_compressed_video` (LCM). The two share
/// their `decode` and `stream_decoder` wiring, so this ought to behave identically — and if it does not, the
/// difference is the wire format and not the robot.
#[tokio::test(flavor = "multi_thread")]
async fn the_robot_s_own_encoding_is_re_encoded_too() -> Result<()> {
    if zenoh_dimos_codecs::utils::h265::Stream::new().is_err() {
        eprintln!("skipping: libde265 is not installed");
        return Ok(());
    }
    let (running, session, url) = start().await;
    let _publisher = keep_publishing_as(&session, "camera/front", compressed_video_lcm);
    let client = Client::connect(&url, ClientOptions::default()).await?;
    let options = SubscribeOptions {
        encoding: Some("dimos_lcm_compressed_video".to_owned()),
        channel: Some("video-h264".to_owned()),
        metadata: Some(true),
        ..Default::default()
    };
    let mut camera = client.subscribe("camera/front", options).await?;
    let (frames, sizes, elapsed) = take_frames(&mut camera, zenoh_gateway::VideoFormat::H264).await;

    // when this fails, the gateway's own account of why is the whole point of asking for it
    let stats = client.stats().await?;
    let channel = stats["channels"].as_array().and_then(|channels| channels.first()).cloned().unwrap_or_default();
    assert!(frames >= 15, "only {frames} re-encoded frames in {elapsed:?}; gateway says {}", channel["stats"]);
    assert!(sizes.iter().all(|size| *size == (1280, 720)), "source sizes {sizes:?}");

    drop(camera);
    client.close().await;
    running.shutdown().await?;
    Ok(())
}

/// A publisher faster than the transcode can keep up with, so the sample queue never empties.
///
/// This is how the robot broke while every test here passed: at 30 Hz from a real camera the queue was never empty,
/// and the send loop skipped encoding whenever anything was waiting — which was always — so it decoded forever and
/// encoded nothing. Zero frames, no error, nothing in the log. The tests above all drain their queue on a fast
/// machine, so none of them could see it.
///
/// What it asserts is only that frames keep coming when the input outruns the encoder. The stream is expected to
/// degrade here — samples drop from the queue and the decoder resyncs — because 250 Hz of 720p is beyond any
/// software transcode; what it must not do is stop.
#[tokio::test(flavor = "multi_thread")]
async fn frames_keep_coming_when_the_publisher_outruns_the_encoder() -> Result<()> {
    if zenoh_dimos_codecs::utils::h265::Stream::new().is_err() {
        eprintln!("skipping: libde265 is not installed");
        return Ok(());
    }
    let (running, session, url) = start().await;
    let keyframe = fixture("compressed/m20_front_keyframe.h265");
    let predicted: Vec<Vec<u8>> = access_units(&fixture("compressed/m20_front_2s.h265")).into_iter().filter(|(keyframe, _)| !keyframe).map(|(_, unit)| unit).collect();
    let flood = {
        let session = session.clone();
        tokio::spawn(async move {
            let frames: Vec<Vec<u8>> = [vec![keyframe], predicted].concat();
            for access_unit in frames.iter().cycle() {
                if session.put("camera/front", compressed_video_lcm(access_unit, "h265")).await.is_err() {
                    return;
                }
                tokio::time::sleep(Duration::from_millis(4)).await;
            }
        })
    };
    let client = Client::connect(&url, ClientOptions::default()).await?;
    let options = SubscribeOptions {
        encoding: Some("dimos_lcm_compressed_video".to_owned()),
        channel: Some("video-h264".to_owned()),
        metadata: Some(true),
        ..Default::default()
    };
    let mut camera = client.subscribe("camera/front", options).await?;
    let (frames, _, elapsed) = take_frames(&mut camera, zenoh_gateway::VideoFormat::H264).await;
    let stats = client.stats().await?;
    let channel = stats["channels"].as_array().and_then(|channels| channels.first()).cloned().unwrap_or_default();
    assert!(frames >= 15, "only {frames} frames in {elapsed:?} with the queue always full; gateway says {}", channel["stats"]);

    flood.abort();
    drop(camera);
    client.close().await;
    running.shutdown().await?;
    Ok(())
}

/// Two cameras transcoded at once on one connection, which is what a browser showing front and rear does: two
/// subscriptions, two stream decoders and two encode sessions, sharing nothing but the connection's pacer. Neither may
/// be starved by the other.
///
/// **This does not reproduce the robot's dual-camera problem**, where the second stream ran at a third of the first's
/// rate. It was written to, and both streams come out within a millisecond of each other here — with openh264 and with
/// x264enc, and with the cores pinned to match. So it is a guard against one stream starving another, not evidence
/// about that failure, and it should not be read as covering it.
#[tokio::test(flavor = "multi_thread")]
async fn two_cameras_transcoded_at_once_both_hold_their_rate() -> Result<()> {
    if zenoh_dimos_codecs::utils::h265::Stream::new().is_err() {
        eprintln!("skipping: libde265 is not installed");
        return Ok(());
    }
    let (running, session, url) = start().await;
    let _front = keep_publishing_as(&session, "camera/front", compressed_video_lcm);
    let _rear = keep_publishing_as(&session, "camera/rear", compressed_video_lcm);
    // one client, as one browser is: both tracks on one peer connection
    let client = Client::connect(&url, ClientOptions::default()).await?;
    let options = || SubscribeOptions {
        encoding: Some("dimos_lcm_compressed_video".to_owned()),
        channel: Some("video-h264".to_owned()),
        metadata: Some(true),
        ..Default::default()
    };
    let mut front = client.subscribe("camera/front", options()).await?;
    let mut rear = client.subscribe("camera/rear", options()).await?;

    let started = Instant::now();
    let (front_side, rear_side) = tokio::join!(
        take_frames(&mut front, zenoh_gateway::VideoFormat::H264),
        take_frames(&mut rear, zenoh_gateway::VideoFormat::H264),
    );
    let elapsed = started.elapsed();
    let (front_frames, _, front_time) = front_side;
    let (rear_frames, _, rear_time) = rear_side;
    let stats = client.stats().await?;
    let per_key = |key: &str| {
        stats["channels"]
            .as_array()
            .and_then(|channels| channels.iter().find(|channel| channel["key"].as_str() == Some(key)))
            .map(|channel| channel["stats"].to_string())
            .unwrap_or_default()
    };
    eprintln!("front: {front_frames} frames in {front_time:?}\n  {}", per_key("camera/front"));
    eprintln!("rear:  {rear_frames} frames in {rear_time:?}\n  {}", per_key("camera/rear"));
    eprintln!("both done in {elapsed:?}");

    // neither may be starved by the other: both are asked for the same number of frames, so both should take about as
    // long. The robot saw the second at a third of the first's rate.
    assert!(front_frames >= 15 && rear_frames >= 15, "front {front_frames}, rear {rear_frames}");
    let (quick, slow) = (front_time.min(rear_time).as_secs_f64(), front_time.max(rear_time).as_secs_f64());
    assert!(slow < quick * 2.0, "one stream took {slow:.1}s for the frames the other got in {quick:.1}s");

    drop(front);
    drop(rear);
    client.close().await;
    running.shutdown().await?;
    Ok(())
}

/// A transcoded stream must not shrink its own picture away over time.
///
/// `reserved()` asks the allocator to reserve a stream's *measured* rate rather than offer it a grant it could not
/// meet, which is right for audio and for a passthrough stream: what they send does not depend on what they are
/// granted. A re-encoded stream is not like that. Its grant sets the encoder's bitrate, the bitrate sets how many
/// bytes it sends, and `usage` then reports those bytes as the whole of its demand - so a picture that shrinks once,
/// for any reason, is granted less, shrinks again, and has nothing pulling it back.
///
/// **This exhibits that only where something supplies the first shrink.** A debug build is slow enough that the CPU
/// governor does, and then the grant stays at a twentieth of what 720p needs; a release build on a fast machine never
/// starts, so it passes either way - checked, by putting the bug back. The guard that actually catches it is
/// `subscription::tests::a_re_encoded_stream_is_never_thinned_but_never_reserved`, which tests the two predicates
/// rather than waiting for a machine slow enough to show the consequence. This stays as the end-to-end witness that
/// a transcoded stream reaches full size at all.
#[tokio::test(flavor = "multi_thread")]
async fn a_transcoded_stream_keeps_its_picture() -> Result<()> {
    if zenoh_dimos_codecs::utils::h265::Stream::new().is_err() {
        eprintln!("skipping: libde265 is not installed");
        return Ok(());
    }
    let (running, session, url) = start().await;
    let _publisher = keep_publishing_as(&session, "camera/front", compressed_video_lcm);
    let client = Client::connect(&url, ClientOptions::default()).await?;
    let options = SubscribeOptions {
        encoding: Some("dimos_lcm_compressed_video".to_owned()),
        channel: Some("video-h264".to_owned()),
        metadata: Some(true),
        ..Default::default()
    };
    let mut camera = client.subscribe("camera/front", options).await?;

    // read frames for a while, sampling what the allocator grants and what size the picture is
    let mut samples = Vec::new();
    for _ in 0..6 {
        let _ = take_frames(&mut camera, zenoh_gateway::VideoFormat::H264).await;
        let stats = client.stats().await?;
        if let Some(channel) = stats["channels"].as_array().and_then(|channels| channels.first()) {
            let number = |name: &str| channel["stats"][name].as_f64().unwrap_or(0.0);
            samples.push((channel["allocation"]["demandBytesPerSec"].as_f64().unwrap_or(0.0), number("videoWidth"), number("videoHeight"), channel["allocation"]["budgetBytesPerSec"].as_f64().unwrap_or(0.0)));
        }
    }
    for (demand, width, height, budget) in &samples {
        eprintln!("asks for {demand:>12.0} B/s   granted {budget:>10.0} B/s   picture {width:>6.0}x{height:.0}");
    }

    // What the fix guarantees is that the grant is not pinned to the bytes the stream is already sending, so the
    // assertion is about the allocator's willingness rather than a picture size - which also depends on how fast the
    // build and the machine are, and would make this test pass or fail for the wrong reasons. 720p at 30 Hz wants
    // ~190 kB/s; the ratchet held this stream at 6-26 kB/s on both a developer machine and the robot, and never
    // climbed. Anything well clear of that could only come from a demand the stream can grow into.
    // The demand, not the grant or the picture: it is what the fix changes, and unlike the other two it does not
    // depend on how fast this machine encodes. While the stream was reserved its demand *was* the bytes it had just
    // sent, so a shrunken picture could only ask for less; now it is priced from the quality it wants, which is far
    // more than a shrunken stream sends and is what lets it climb back. 720p at 30 Hz wants ~190 kB/s.
    let best = samples.iter().map(|(demand, ..)| *demand).fold(0.0, f64::max);
    assert!(best > 500_000.0, "the stream never asked for more than {best:.0} B/s, so it cannot grow into 720p: {samples:?}");
    drop(camera);
    client.close().await;
    running.shutdown().await?;
    Ok(())
}
