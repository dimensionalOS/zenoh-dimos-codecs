//! An LCM reader for dimos messages over zenoh: an 8-byte type fingerprint, then big-endian packed fields.
//! Zero-copy over the sample payload.

use anyhow::{Context, Result, ensure};

/// Reads LCM: big endian, packed, strings as `i32 length (incl. NUL) | bytes | NUL`.
pub(crate) struct Lcm<'a> {
    body: &'a [u8],
    position: usize,
}

impl<'a> Lcm<'a> {
    pub(crate) fn new(payload: &'a [u8], fingerprint: [u8; 8], type_name: &str) -> Result<Self> {
        ensure!(payload.len() >= 8, "LCM payload shorter than its fingerprint");
        ensure!(payload[..8] == fingerprint, "not an LCM {type_name} (fingerprint {:02x?})", &payload[..8]);
        Ok(Lcm { body: payload, position: 8 })
    }

    pub(crate) fn take(&mut self, length: usize) -> Result<&'a [u8]> {
        let end = self.position.checked_add(length).context("LCM length overflow")?;
        ensure!(end <= self.body.len(), "LCM message truncated (need {end} bytes, have {})", self.body.len());
        let bytes = &self.body[self.position..end];
        self.position = end;
        Ok(bytes)
    }

    pub(crate) fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }

    pub(crate) fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_be_bytes(self.take(4)?.try_into()?))
    }

    pub(crate) fn length(&mut self) -> Result<usize> {
        let value = self.i32()?;
        usize::try_from(value).context("negative LCM length")
    }

    pub(crate) fn string(&mut self) -> Result<String> {
        let length = self.length()?;
        let bytes = self.take(length)?;
        Ok(String::from_utf8_lossy(bytes.strip_suffix(&[0]).unwrap_or(bytes)).into_owned())
    }

    /// std_msgs.Header: seq, stamp and frame_id
    pub(crate) fn header(&mut self) -> Result<crate::codecs::msgs::Header> {
        let seq = self.i32()? as u32;
        let (sec, nsec) = (self.i32()?, self.i32()?);
        Ok(crate::codecs::msgs::Header::new(sec, nsec as u32, self.string()?, Some(seq)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fingerprint_lengths_and_bad_input() {
        let fingerprint = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut payload = fingerprint.to_vec();
        payload.extend(3i32.to_be_bytes());
        payload.extend(b"ab\0");
        assert_eq!(Lcm::new(&payload, fingerprint, "t").unwrap().string().unwrap(), "ab");
        assert!(Lcm::new(&payload, [0; 8], "t").is_err(), "another type's fingerprint");
        assert!(Lcm::new(&payload[..5], fingerprint, "t").is_err(), "shorter than the fingerprint");
        let mut negative = fingerprint.to_vec();
        negative.extend((-1i32).to_be_bytes());
        assert!(Lcm::new(&negative, fingerprint, "t").unwrap().length().is_err(), "a negative length");
        assert!(Lcm::new(&payload[..12], fingerprint, "t").unwrap().string().is_err(), "a string past the payload");
    }
}
