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
    let server = Server::builder().session(session.clone()).encoding(Ros2CompressedVideo).build().await.unwrap();
    let running = server.bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", running.local_addr());
    (running, session, url)
}

/// Publishes the robot's capture on `key` at 30 Hz until dropped: the keyframe with its parameter sets, then the
/// predicted frames that follow it, over and over.
fn keep_publishing(session: &zenoh::Session, key: &str) -> tokio::task::JoinHandle<()> {
    let keyframe = fixture("compressed/m20_front_keyframe.h265");
    let predicted: Vec<Vec<u8>> = access_units(&fixture("compressed/m20_front_2s.h265")).into_iter().filter(|(keyframe, _)| !keyframe).map(|(_, unit)| unit).collect();
    assert!(predicted.len() > 20, "the capture has {} predicted frames", predicted.len());
    let (session, key) = (session.clone(), key.to_owned());
    tokio::spawn(async move {
        let frames: Vec<Vec<u8>> = [vec![keyframe], predicted].concat();
        for access_unit in frames.iter().cycle() {
            if session.put(&key, compressed_video_cdr(access_unit, "h265")).await.is_err() {
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
    // and no sample was dropped from under the decoder, which would have cost a resync
    let stats = client.stats().await?;
    let resyncs = stats["channels"].as_array().and_then(|channels| channels.first()).map(|channel| channel["stats"]["streamResyncs"].as_u64().unwrap_or(0)).unwrap_or(0);
    assert_eq!(resyncs, 0, "the decoder had to start over {resyncs} times; stats: {}", stats["channels"]);

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
