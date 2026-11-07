//! Replays recorded `CompressedVideo` samples (a dump of `f64 unix seconds | u32 length | payload`, little endian)
//! through a codec's stream decoder: pictures, skips, errors and decode time.
//! `cargo run --release --example stream_replay -- <dimos_lcm_compressed_video|ros2_compressed_video> <dump> [drop every N]`

use std::time::Instant;
use zenoh_gateway::{Channel, DecodedFrame, EncodingSample, VideoFormat};

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let (name, dump) = (&args[1], std::fs::read(&args[2])?);
    let drop_every: Option<usize> = args.get(3).map(|every| every.parse()).transpose()?;
    let codec = zenoh_dimos_codecs::codecs::all().into_iter().find(|codec| codec.name() == name).ok_or_else(|| anyhow::anyhow!("no codec {name}"))?;
    let mut decoder = codec.stream_decoder().ok_or_else(|| anyhow::anyhow!("{name} has no stream decoder"))?;
    let encoding = zenoh_gateway::zenoh::bytes::Encoding::default();
    let (mut at, mut index) = (0, 0);
    let (mut pictures, mut skips, mut errors, mut dropped, mut decode_ms) = (0, 0, 0, 0, Vec::new());
    while at + 12 <= dump.len() {
        let length = u32::from_le_bytes(dump[at + 8..at + 12].try_into()?) as usize;
        let payload = &dump[at + 12..at + 12 + length];
        at += 12 + length;
        index += 1;
        if drop_every.is_some_and(|every| index % every == 0) {
            dropped += 1;
            continue;
        }
        let started = Instant::now();
        let fed = decoder.feed(&EncodingSample::new("k", payload, &encoding), Channel::Video(VideoFormat::H264));
        decode_ms.push(started.elapsed().as_secs_f64() * 1000.0);
        match fed {
            Ok(Some(DecodedFrame::Video(image))) => {
                if pictures == 0 {
                    println!("first picture at sample {index}: {}x{}", image.width(), image.height());
                }
                pictures += 1;
            }
            Ok(_) => skips += 1,
            Err(error) => {
                errors += 1;
                println!("sample {index}: {error:#}");
            }
        }
    }
    decode_ms.sort_by(f64::total_cmp);
    let percentile = |share: f64| decode_ms[((decode_ms.len() - 1) as f64 * share) as usize];
    println!("{index} samples ({dropped} dropped): {pictures} pictures, {skips} nothing to show, {errors} errors; decode p50 {:.1} ms p95 {:.1} ms max {:.1} ms", percentile(0.5), percentile(0.95), percentile(1.0));
    Ok(())
}
