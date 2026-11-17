//! `foxglove_msgs/RawAudio` from dimos (LCM over zenoh) as audio: `pcm-s16` blocks to PCM for zenoh-gateway's Opus track.

use super::RawAudio;
use crate::utils::{lcm::Lcm, pcm};
use anyhow::Result;
use zenoh_gateway::{MessageEncoding, Channel, EncodingOutput, EncodingSample, DecodedFrame};

/// The LCM fingerprint of dimos-lcm's type, as `lcm-gen` computes it.
const LCM_RAW_AUDIO: [u8; 8] = [0x28, 0xe2, 0x3a, 0xc0, 0x24, 0xcb, 0x1c, 0x86];

/// `dimos_lcm_raw_audio`
pub struct DimosLcmRawAudio;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<RawAudio<'_>> {
    let mut lcm = Lcm::new(payload, LCM_RAW_AUDIO, "foxglove_msgs.RawAudio")?;
    let data_length = lcm.length()?;
    // builtin_interfaces.Time timestamp
    let (sec, nsec) = (lcm.i32()?, lcm.i32()?);
    let data = lcm.take(data_length)?;
    let format = lcm.string()?;
    let sample_rate = lcm.i32()? as u32;
    let channels = lcm.i32()? as u32;
    Ok(RawAudio { header: super::Header::new(sec, nsec as u32, String::new(), None), data, format, sample_rate, channels })
}

impl MessageEncoding for DimosLcmRawAudio {
    fn name(&self) -> &str {
        "dimos_lcm_raw_audio"
    }

    fn output(&self) -> EncodingOutput {
        EncodingOutput::Audio
    }

    /// the message's timestamp, which the gateway sends with each sample's frame
    fn header(&self, sample: &EncodingSample<'_>) -> Option<zenoh_gateway::MessageHeader> {
        parse(sample.payload).ok().map(|message| message.header.to_gateway())
    }

    fn decode(&self, sample: &EncodingSample<'_>, _channel: Channel) -> Result<DecodedFrame> {
        let audio = parse(sample.payload)?;
        Ok(DecodedFrame::Audio(pcm::s16_to_pcm(audio.data, &audio.format, audio.sample_rate, audio.channels)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses() {
        // LCM: fingerprint, data_length, Time, data (2 samples), format, rate, channels
        let mut payload = LCM_RAW_AUDIO.to_vec();
        payload.extend(4i32.to_be_bytes());
        payload.extend([0u8; 8]);
        payload.extend([1, 0, 2, 0]);
        payload.extend(8i32.to_be_bytes());
        payload.extend(b"pcm-s16\0");
        payload.extend(16_000i32.to_be_bytes());
        payload.extend(1i32.to_be_bytes());
        let audio = parse(&payload).unwrap();
        assert_eq!((audio.data, audio.format.as_str(), audio.sample_rate, audio.channels), (&[1, 0, 2, 0][..], "pcm-s16", 16_000, 1));
    }
}
