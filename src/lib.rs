//! zenoh-dimos-codecs: [zenoh-gateway](https://github.com/jeff-hykin/zenoh-gateway) codecs for ROS 2
//! (rmw_zenoh, CDR) and dimos (LCM) sensor messages, implemented through zenoh-gateway's public
//! [`MessageEncoding`](zenoh_gateway::MessageEncoding) trait like any external one.
//!
//! ```no_run
//! # async fn run() -> anyhow::Result<()> {
//! let mut builder = zenoh_gateway::Server::builder();
//! for codec in zenoh_dimos_codecs::all() {
//!     builder = builder.shared_encoding(codec);
//! }
//! let server = builder.build().await?;
//! # Ok(())
//! # }
//! ```
//!
//! One codec per message type, in [`codecs::msgs`] (e.g. [`codecs::msgs::Ros2CompressedImage`]); the input type
//! decides the output:
//! - `*_image`, `*_compressed_image`: color/mono images, H.264 on a WebRTC video track
//! - `*_compressed_video`: H.264 or H.265 keyframes (foxglove `CompressedVideo`, each with its parameter sets), decoded
//!   (H.265 through libde265, loaded at runtime) and re-encoded like any picture, so every browser can play them
//! - `*_depth`, `*_compressed_depth`: lossless depth (u16/f32) as zenoh-gateway fields
//! - `*_pointcloud2`: thinned (every Nth point) + int16 quantized points as zenoh-gateway fields
//! - `*_raw_audio`: `pcm-s16` audio blocks, Opus on a WebRTC audio track
//!
//! Depth and point clouds are zstd-compressed by default ([`MessageEncoding::default_compress`](zenoh_gateway::MessageEncoding::default_compress)); zenoh-gateway's
//! client decodes them into `msg.decoded` with no codec code in the page.

pub mod codecs;
/// Hardware H.264 encoders (features `videotoolbox`, `gstreamer`) with a fallback to software, for
/// [`ServerBuilder::video_encoder`](zenoh_gateway::ServerBuilder::video_encoder).
pub mod encoders;
/// JPEG decoding on the Jetson's hardware decoder, in a child process (feature `gstreamer`), for the video path.
#[cfg(feature = "gstreamer")]
pub mod hardware_jpeg;

/// What a deployment chooses about how video is decoded and encoded.
pub mod settings;
/// What the codecs share: wire readers (CDR, LCM), image decoders (JPEG, PNG, WebP, JPEG XL, raw pixels), and the
/// depth and point-cloud encodings.
pub mod utils;

pub use codecs::all;
