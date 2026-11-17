//! Raw audio (foxglove `RawAudio` blocks) to the PCM zenoh-gateway's Opus path takes: 16-bit, 1 or 2 channels, at a
//! rate Opus accepts.

use anyhow::{Result, ensure};
use zenoh_gateway::AudioPcm;

/// The rates Opus takes (RFC 6716); others are resampled to `RESAMPLED_RATE`.
const OPUS_RATES: [u32; 5] = [8_000, 12_000, 16_000, 24_000, 48_000];
/// Opus's full-band rate, the one it codes at internally.
const RESAMPLED_RATE: u32 = 48_000;

/// An interleaved little-endian `pcm-s16` block as Opus-ready PCM: channels past the second are dropped, and a rate
/// Opus doesn't take is resampled to 48 kHz (linear, per block, so a block boundary can carry a tiny step).
pub fn s16_to_pcm(data: &[u8], format: &str, sample_rate: u32, channels: u32) -> Result<AudioPcm> {
    ensure!(format == "pcm-s16", "audio format {format:?} is not supported (pcm-s16 only)");
    ensure!(channels > 0 && sample_rate > 0, "audio block with {channels} channels at {sample_rate} Hz");
    let samples: Vec<i16> = data.as_chunks::<2>().0.iter().map(|&bytes| i16::from_le_bytes(bytes)).collect();
    let channels = channels as usize;
    ensure!(samples.len().is_multiple_of(channels), "{} samples are not whole {channels}-channel frames", samples.len());
    let kept = channels.min(2);
    let frames: Vec<&[i16]> = samples.chunks_exact(channels).map(|frame| &frame[..kept]).collect();
    if OPUS_RATES.contains(&sample_rate) {
        return AudioPcm::new(sample_rate, kept as u8, frames.concat());
    }
    let out_frames = (frames.len() as u64 * RESAMPLED_RATE as u64 / sample_rate as u64) as usize;
    let mut out = Vec::with_capacity(out_frames * kept);
    for index in 0..out_frames {
        let position = index as f64 * sample_rate as f64 / RESAMPLED_RATE as f64;
        let (before, fraction) = (position.floor() as usize, position.fract());
        let after = (before + 1).min(frames.len() - 1);
        for (&a, &b) in frames[before].iter().zip(frames[after]) {
            let (a, b) = (a as f64, b as f64);
            out.push((a + (b - a) * fraction).round() as i16);
        }
    }
    AudioPcm::new(RESAMPLED_RATE, kept as u8, out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s16(samples: &[i16]) -> Vec<u8> {
        samples.iter().flat_map(|sample| sample.to_le_bytes()).collect()
    }

    #[test]
    fn opus_rates_pass_through_and_extra_channels_drop() {
        let pcm = s16_to_pcm(&s16(&[1, 2, 3, 4, 5, 6]), "pcm-s16", 48_000, 3).unwrap();
        assert_eq!((pcm.sample_rate(), pcm.channels(), pcm.samples()), (48_000, 2, &[1, 2, 4, 5][..]));
        assert!(s16_to_pcm(&s16(&[1, 2]), "pcm-f32", 48_000, 1).is_err());
        assert!(s16_to_pcm(&s16(&[1, 2, 3]), "pcm-s16", 48_000, 2).is_err(), "half a frame");
        assert!(s16_to_pcm(&s16(&[1, 2]), "pcm-s16", 48_000, 0).is_err(), "no channels");
        assert!(s16_to_pcm(&s16(&[1, 2]), "pcm-s16", 0, 1).is_err(), "no rate");
    }

    #[test]
    fn other_rates_resample_to_48k() {
        // 441 frames at 44.1 kHz = 10 ms = 480 frames at 48 kHz
        let pcm = s16_to_pcm(&s16(&vec![100; 441]), "pcm-s16", 44_100, 1).unwrap();
        assert_eq!((pcm.sample_rate(), pcm.channels(), pcm.samples().len()), (48_000, 1, 480));
        assert!(pcm.samples().iter().all(|&sample| sample == 100));
    }
}
