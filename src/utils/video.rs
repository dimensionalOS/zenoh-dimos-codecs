//! What the two compressed-video encodings share, which is everything except how their message is parsed.
//!
//! `ros2_compressed_video` (CDR) and `dimos_lcm_compressed_video` (LCM) carry the same
//! `foxglove_msgs/CompressedVideo` and want the same decisions made about it. Those decisions lived in both files,
//! which is how the H.264 relay came to be added to one of them and nearly not the other; they live here now and each
//! encoding is the wire reader plus four one-line delegations.

use super::bitstream;
use super::video_passthrough::{H264Relay, Passthrough};
use super::video_stream::{CompressedVideoStream, Parse};
use anyhow::Result;
use zenoh_gateway::{Channel, DecodedFrame, EncodingSample, StreamDecoder, VideoEncoder, VideoFormat};

/// One sample for `channel`: the coded unit itself when the track wants the format the publisher sent, else the picture
/// in it.
///
/// **Which of the two depends on the pair**, not on the track alone. A track whose format matches the source needs no
/// transcode at all — the receiver's own decoder keeps the reference chain, which is both free and the only way every
/// frame plays rather than one keyframe a second.
pub fn decode(parse: Parse, sample: &EncodingSample<'_>, channel: Channel) -> Result<DecodedFrame> {
    let message = parse(sample.payload)?;
    let source = bitstream::by_name(&message.format)?;
    if matches!(channel, Channel::Video(track) if bitstream::passes_through(source.format(), track)) {
        return Ok(DecodedFrame::data(source.unit(message.data)?));
    }
    // decoded alone, with no earlier frame to predict from: only a keyframe carrying its own headers can be, which is
    // what `check_decodable_alone` says in the publisher's terms. A stream of predicted frames needs `stream_decoder`.
    bitstream::check_decodable_alone(source, message.data)?;
    Ok(DecodedFrame::Video(message.header.add_to_video(source.decode_alone(message.data)?)))
}

/// The encoder for a video track: one that forwards a source already in `format` and encodes anything else.
pub fn video_encoder(format: VideoFormat) -> Option<Box<dyn VideoEncoder>> {
    match format {
        // H.265 is only ever forwarded: there is no H.265 encoder here, so a source in any other format cannot reach
        // this track at all (and the gateway's `passthrough_only` says the same of it)
        VideoFormat::H265 => Some(Box::new(Passthrough::new(VideoFormat::H265)) as Box<dyn VideoEncoder>),
        // H.264 forwards an H.264 source unchanged, else encodes a decoded picture
        VideoFormat::H264 => Some(Box::new(H264Relay::new()) as Box<dyn VideoEncoder>),
        _ => None,
    }
}

/// A stream decoder for a video channel, which **every** video channel gets.
///
/// It is what keeps the state a predicted frame needs, and also what asks the gateway not to thin the stream — and a
/// forwarded stream must not be thinned either, since dropping a frame other frames predict from breaks the picture
/// until the publisher's next keyframe. One path for both is therefore not just tidier: it is the same requirement.
pub fn stream_decoder(parse: Parse, channel: Channel) -> Option<Box<dyn StreamDecoder>> {
    let Channel::Video(track) = channel else { return None };
    Some(Box::new(CompressedVideoStream::new(parse, track)) as Box<dyn StreamDecoder>)
}
