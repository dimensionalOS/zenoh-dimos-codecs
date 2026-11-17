//! Compressed images (`sensor_msgs/CompressedImage` data, or a dimos Image carrying a file): the format from the
//! bytes, then that format's decoder.

use crate::codecs::msgs::Header;
use crate::utils::raw_pixels::Rgb8;
use crate::utils::{jpeg, jxl, png, webp};
use anyhow::{Result, bail};
use serde_json::{Map, Value};
use zenoh_gateway::{Channel, EncodeOptions, EncodingOutput, VideoImage};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileFormat {
    Jpeg,
    Png,
    Webp,
    Jxl,
}

impl FileFormat {
    pub fn as_str(self) -> &'static str {
        match self {
            FileFormat::Jpeg => "jpeg",
            FileFormat::Png => "png",
            FileFormat::Webp => "webp",
            FileFormat::Jxl => "jxl",
        }
    }
}

/// The file format, from magic bytes first and the message's format string second.
pub fn sniff_format(data: &[u8], hint: &str) -> Result<FileFormat> {
    if data.starts_with(&[0xff, 0xd8, 0xff]) {
        return Ok(FileFormat::Jpeg);
    }
    if data.starts_with(&[0x89, b'P', b'N', b'G']) {
        return Ok(FileFormat::Png);
    }
    if data.len() >= 12 && &data[0..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        return Ok(FileFormat::Webp);
    }
    if data.starts_with(&[0xff, 0x0a]) || data.starts_with(&[0, 0, 0, 0x0c, b'J', b'X', b'L', b' ']) {
        return Ok(FileFormat::Jxl);
    }
    let hint = hint.to_ascii_lowercase();
    for (name, format) in [("jpeg", FileFormat::Jpeg), ("jpg", FileFormat::Jpeg), ("png", FileFormat::Png), ("webp", FileFormat::Webp), ("jxl", FileFormat::Jxl)] {
        if hint.contains(name) {
            return Ok(format);
        }
    }
    bail!("unrecognized compressed image (format {hint:?}, magic {:02x?})", &data[..data.len().min(8)])
}

/// What a picture encoding sends on the data channel (`encodeOptions.format`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileWanted {
    /// the message's own file, untouched (compressed images only; the default there)
    Passthrough,
    /// a JPEG at the allocator's quality (the file itself when it already is one and the quality is 1)
    Jpeg,
    /// a PNG (lossless; the file itself when it already is one)
    Png,
}

impl FileWanted {
    /// `encodeOptions.format`, checked: `passthrough` only where `passthrough_default` (a file to pass), and no other option
    /// but `header` ([`header_wanted`]).
    pub fn from_options(options: &Map<String, Value>, passthrough_default: bool) -> Result<Self, String> {
        if let Some(other) = options.keys().find(|key| *key != "format" && *key != "header") {
            return Err(format!("unknown encodeOptions {other:?} (format and header only)"));
        }
        Ok(match (options.get("format").and_then(Value::as_str), options.get("format")) {
            (Some("passthrough"), _) | (None, None) if passthrough_default => FileWanted::Passthrough,
            (Some("jpeg"), _) | (None, None) => FileWanted::Jpeg,
            (Some("png"), _) => FileWanted::Png,
            (_, Some(other)) => return Err(format!("encodeOptions.format must be {}, got {other}", if passthrough_default { "\"passthrough\", \"jpeg\" or \"png\"" } else { "\"jpeg\" or \"png\"" })),
            (_, None) => unreachable!("handled above"),
        })
    }
}

/// `encodeOptions.header` (default false): the file goes inside a fields message with its message's header,
/// `{stamp, frameId, format, data}`, instead of as bare bytes (which have no room for one).
pub fn header_wanted(options: &Map<String, Value>) -> Result<bool, String> {
    match options.get("header") {
        None => Ok(false),
        Some(Value::Bool(wanted)) => Ok(*wanted),
        Some(other) => Err(format!("encodeOptions.header must be true or false, got {other}")),
    }
}

/// The data channel's bytes for `file`: the file itself, or with `encodeOptions.header` a fields message
/// `{stamp, frameId, format (jpeg, png, webp or jxl), data (the file)}`.
pub fn with_header(file: Vec<u8>, header: &Header, options: &EncodeOptions) -> Result<Vec<u8>> {
    if !header_wanted(&options.options).map_err(anyhow::Error::msg)? {
        return Ok(file);
    }
    let format = sniff_format(&file, "")?;
    Ok(header.add_to(zenoh_gateway::Fields::new()).text("format", format.as_str()).array("data", &file).build())
}

/// [`MessageEncoding::output_on`](zenoh_gateway::MessageEncoding::output_on) for a picture encoding: pictures on any video
/// channel (no options), files on the data channel ([`FileWanted`]; fields with [`header_wanted`]).
pub fn output_on(name: &str, channel: Channel, options: &Map<String, Value>, passthrough_default: bool) -> Result<EncodingOutput, String> {
    match channel {
        Channel::Video(_) if options.is_empty() => Ok(EncodingOutput::Video),
        Channel::Video(_) => Err(format!("{name} takes no encodeOptions on video channels but quality")),
        Channel::Data => FileWanted::from_options(options, passthrough_default).and_then(|_| header_wanted(options)).map(|header| if header { EncodingOutput::Fields } else { EncodingOutput::Data }),
        Channel::Audio => Err(format!("{name} sends pictures, not audio")),
    }
}

/// A compressed image as the data channel's decoded form: the file, decoded only if a conversion needs it.
pub struct ImageFile {
    pub data: Vec<u8>,
    pub format: FileFormat,
}

impl ImageFile {
    pub fn new(data: &[u8], hint: &str) -> Result<Self> {
        Ok(ImageFile { format: sniff_format(data, hint)?, data: data.to_vec() })
    }

    /// The file the subscription asked for (`options.format`) at `options.quality`.
    pub fn encode(&self, options: &EncodeOptions) -> Result<Vec<u8>> {
        let wanted = FileWanted::from_options(&options.options, true).map_err(anyhow::Error::msg)?;
        Ok(match wanted {
            FileWanted::Passthrough => self.data.clone(),
            FileWanted::Jpeg if self.format == FileFormat::Jpeg && options.quality >= 0.999 => self.data.clone(),
            FileWanted::Png if self.format == FileFormat::Png => self.data.clone(),
            FileWanted::Jpeg => jpeg::encode(&to_rgb(&self.data, "")?, options.quality)?,
            FileWanted::Png => png::encode(&to_rgb(&self.data, "")?)?,
        })
    }
}

/// RGB8 as the file `options.format` asks for (raw images: `jpeg`, the default, or `png`).
pub fn encode_rgb(image: &Rgb8, options: &EncodeOptions) -> Result<Vec<u8>> {
    match FileWanted::from_options(&options.options, false).map_err(anyhow::Error::msg)? {
        FileWanted::Png => png::encode(image),
        _ => jpeg::encode(image, options.quality),
    }
}

/// The allocator's size prior for a picture file of `payload_bytes` with `options`: passed or PNG ≈ the payload, JPEG
/// shrinking with quality.
pub fn estimated_bytes(payload_bytes: usize, options: &EncodeOptions, passthrough_default: bool) -> f64 {
    let payload = payload_bytes as f64;
    match FileWanted::from_options(&options.options, passthrough_default) {
        Ok(FileWanted::Jpeg) => payload * (0.1 + 0.9 * options.quality.clamp(0.0, 1.0)),
        _ => payload,
    }
}

/// [`estimated_bytes`] for raw pixels (raw images' codecs): a JPEG of them is a tenth of them or less.
pub(crate) fn raw_estimated_bytes(payload_bytes: usize, options: &EncodeOptions) -> f64 {
    estimated_bytes(payload_bytes, options, false) * 0.1
}

/// A compressed image (any [`FileFormat`]) as RGB8.
pub fn to_rgb(data: &[u8], hint: &str) -> Result<Rgb8> {
    match sniff_format(data, hint)? {
        FileFormat::Jpeg => jpeg::to_rgb(data),
        FileFormat::Png => png::to_rgb(data),
        FileFormat::Webp => webp::to_rgb(data),
        FileFormat::Jxl => jxl::to_rgb(data),
    }
}

/// A compressed image for the video path. Once the host enabled the hardware decoder, a JPEG it takes stays a JPEG
/// ([`VideoImage::jpeg`]): the hardware encoder decodes, scales and encodes it in one pipeline, and anything that reads
/// its pixels gets [`decode_video`]'s, made then. The rest are decoded now, by [`decode_video`].
pub fn to_video(data: &[u8], hint: &str) -> Result<VideoImage> {
    #[cfg(feature = "gstreamer")]
    if crate::hardware_jpeg::passthrough()
        && sniff_format(data, hint)? == FileFormat::Jpeg
        && let Some((width, height)) = crate::hardware_jpeg::hardware_size(data)
    {
        return VideoImage::jpeg(width, height, data.to_vec(), decode_jpeg);
    }
    decode_video(data, hint)
}

/// A JPEG's picture, for [`VideoImage::jpeg`].
pub fn decode_jpeg(data: &[u8]) -> Result<VideoImage> {
    decode_video(data, "jpeg")
}

/// A compressed image's picture. A JPEG the hardware decoder takes goes to it once the host enabled it
/// ([`hardware_jpeg`](crate::hardware_jpeg)); else a YCbCr JPEG with even sides is decoded to I420 without
/// ever becoming RGB: the encoder wants YUV anyway, and the YCbCr -> RGB -> YUV round trip was
/// over half the decode on an ARM core (37 ms of a 1920x1536 frame on a Jetson Orin, against 16).
/// Anything else decodes to RGB.
pub fn decode_video(data: &[u8], hint: &str) -> Result<VideoImage> {
    let format = sniff_format(data, hint)?;
    #[cfg(feature = "gstreamer")]
    if format == FileFormat::Jpeg
        && let Some(decoded) = crate::hardware_jpeg::decode(data)
    {
        match decoded {
            Ok(image) => return Ok(image),
            Err(error) => log::warn!("hardware jpeg decode: {error:#}: this one in software"),
        }
    }
    if format == FileFormat::Jpeg
        && let Some(image) = jpeg::to_i420(data)?
    {
        return Ok(image);
    }
    to_rgb(data, hint)?.into_video()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codecs::msgs::{CompressedImage, dimos_lcm_compressed_image, ros2_compressed_image};
    use crate::utils::raw_pixels::tests::assert_pattern;
    use crate::utils::tests::fixture;

    fn options(json: Value, quality: f64) -> EncodeOptions {
        EncodeOptions { quality, options: serde_json::from_value(json).unwrap() }
    }

    #[test]
    fn files_pass_or_convert() {
        let png_file = fixture("compressed/pattern.png");
        let file = ImageFile::new(&png_file, "png").unwrap();
        assert_eq!(file.encode(&options(serde_json::json!({}), 0.3)).unwrap(), png_file, "passthrough by default, whatever the quality");
        assert_eq!(file.encode(&options(serde_json::json!({"format": "png"}), 1.0)).unwrap(), png_file, "already a png");
        let high = file.encode(&options(serde_json::json!({"format": "jpeg"}), 1.0)).unwrap();
        let low = file.encode(&options(serde_json::json!({"format": "jpeg"}), 0.0)).unwrap();
        assert_eq!(sniff_format(&high, "").unwrap(), FileFormat::Jpeg, "png → jpeg");
        assert!(low.len() < high.len(), "a squeezed jpeg is smaller: {} vs {}", low.len(), high.len());
        crate::utils::raw_pixels::tests::assert_pattern(&to_rgb(&high, "").unwrap(), 8.0, "the converted jpeg");
        let jpeg_file = ImageFile::new(&high, "jpeg").unwrap();
        assert_eq!(jpeg_file.encode(&options(serde_json::json!({"format": "jpeg"}), 1.0)).unwrap(), high, "a jpeg at full quality passes");
        assert_eq!(sniff_format(&jpeg_file.encode(&options(serde_json::json!({"format": "png"}), 1.0)).unwrap(), "").unwrap(), FileFormat::Png, "jpeg → png");
        assert!(FileWanted::from_options(&serde_json::from_value(serde_json::json!({"format": "gif"})).unwrap(), true).is_err());
        assert!(FileWanted::from_options(&serde_json::from_value(serde_json::json!({"format": "passthrough"})).unwrap(), false).is_err(), "raw images have no file to pass");
        assert!(FileWanted::from_options(&serde_json::from_value(serde_json::json!({"size": 2})).unwrap(), true).is_err());
        assert!(output_on("x", Channel::Video(zenoh_gateway::VideoFormat::Av1), &Map::new(), true).is_ok());
        assert!(output_on("x", Channel::Video(zenoh_gateway::VideoFormat::H264), &serde_json::from_value(serde_json::json!({"format": "png"})).unwrap(), true).is_err());
    }

    type Parse = fn(&[u8]) -> Result<CompressedImage<'_>>;

    #[test]
    fn compressed_formats() {
        for (format, tolerance) in [("jpeg", 8.0), ("png", 0.0), ("webp", 0.0), ("jxl", 8.0)] {
            for (parse, file) in [(ros2_compressed_image::parse as Parse, format!("ros2/compressed_{format}.cdr")), (dimos_lcm_compressed_image::parse, format!("dimos/compressed_{format}.bin"))] {
                let payload = fixture(&file);
                let message = parse(&payload).unwrap();
                assert_pattern(&to_rgb(message.data, &message.format).unwrap(), tolerance, &file);
            }
        }
    }
}
