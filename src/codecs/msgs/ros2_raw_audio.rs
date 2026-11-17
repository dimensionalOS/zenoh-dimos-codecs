//! `foxglove_msgs/RawAudio` from ROS 2 (rmw_zenoh, CDR) as audio: `pcm-s16` blocks to PCM for zenoh-gateway's Opus track.

use super::RawAudio;
use crate::utils::{cdr::Cdr, pcm};
use anyhow::Result;
use zenoh_gateway::{MessageEncoding, Channel, EncodingOutput, EncodingSample, DecodedFrame};

/// `ros2_raw_audio`
pub struct Ros2RawAudio;

/// Parses the message, zero-copy over `payload`.
pub fn parse(payload: &[u8]) -> Result<RawAudio<'_>> {
    let mut cdr = Cdr::new(payload)?;
    // builtin_interfaces/Time timestamp
    let (sec, nsec) = (cdr.u32()? as i32, cdr.u32()?);
    let data = cdr.byte_sequence()?;
    let format = cdr.string()?;
    let sample_rate = cdr.u32()?;
    let channels = cdr.u32()?;
    Ok(RawAudio { header: super::Header::new(sec, nsec, String::new(), None), data, format, sample_rate, channels })
}

impl MessageEncoding for Ros2RawAudio {
    fn name(&self) -> &str {
        "ros2_raw_audio"
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
        // CDR little endian: encapsulation, Time, data (2 samples), format, rate, channels
        let mut payload = vec![0, 1, 0, 0];
        payload.extend([0u8; 8]);
        payload.extend(4u32.to_le_bytes());
        payload.extend([1, 0, 2, 0]);
        payload.extend(8u32.to_le_bytes());
        payload.extend(b"pcm-s16\0");
        payload.extend(16_000u32.to_le_bytes());
        payload.extend(1u32.to_le_bytes());
        let audio = parse(&payload).unwrap();
        assert_eq!((audio.data, audio.format.as_str(), audio.sample_rate, audio.channels), (&[1, 0, 2, 0][..], "pcm-s16", 16_000, 1));
    }
}
