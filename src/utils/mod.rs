//! What the message codecs share: the two wire readers, the image decoders, raw audio, and the depth and point-cloud
//! encodings.

pub mod cdr;
pub mod compressed_image;
pub mod compressed_video;
pub mod depth;
pub mod h264;
pub mod h265;
pub mod h265_sps;
pub mod jpeg;
pub mod jxl;
pub mod lcm;
pub mod pcm;
pub mod png;
pub mod pointcloud;
pub mod raw_pixels;
pub mod video_passthrough;
pub mod video_stream;
pub mod webp;

#[cfg(test)]
pub mod tests {
    use std::path::PathBuf;

    pub fn fixture(name: &str) -> Vec<u8> {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("test/fixtures").join(name);
        std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
    }
}


