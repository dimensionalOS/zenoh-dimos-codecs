# zenoh-dimos-codecs

[zenoh-gateway](https://github.com/dimensionalOS/zenoh-gateway) codecs for ROS 2 (rmw_zenoh) and dimos sensor
messages: images as H.264 video, lossless depth and quantized point clouds, plus hardware H.264 encoders
(see [Hardware encoders](#hardware-encoders)). A Rust crate of `zenoh_gateway::MessageEncoding`s and encoders: depth and point clouds are zenoh-gateway **fields**, which
zenoh-gateway's own browser client decodes, so pages need no code from here. The `zenoh-gateway` command
([zenoh-gateway-cli](https://github.com/dimensionalOS/zenoh-gateway-cli)) has them all.

```toml
[dependencies]
zenoh-dimos-codecs = { git = "https://github.com/dimensionalOS/zenoh-dimos-codecs", rev = "<commit>" }
```

```rust
let mut builder = zenoh_gateway::Server::builder();
for codec in zenoh_dimos_codecs::all() {
    builder = builder.shared_encoding(codec);
}
```

```js
import { connect } from "./zenoh_gateway.js" // zenoh-gateway's client/zenoh_gateway.ts, bundled or vendored (see its SPEC.md, "Client")
const z = await connect("http://robot.local:7448")
z.encodings   // [{ name: "dimos_lcm_depth", output: "fields" }, { name: "dimos_lcm_image", output: "video" }, ...]
z.subscribe("dimos/camera/depth/sensor_msgs.Image", { encoding: "dimos_lcm_depth" }, (msg) => draw(msg.decoded))
// a compressed image as a JPEG file on the data channel (converted from PNG if need be), smaller when squeezed:
z.subscribe("camera/sensor_msgs.CompressedImage", { encoding: "dimos_lcm_compressed_image", channel: "data", encodeOptions: { format: "jpeg", quality: 0.8 } },
    async (msg) => draw(await createImageBitmap(new Blob([msg.bytes]))))
```

## Codecs

One file per message type in [`src/codecs/msgs/`](src/codecs/msgs), and the codec's name is the file's name
(`<protocol>_<message type>`; `ros2` is ROS 2 over rmw_zenoh, `dimos_lcm` is dimos' LCM over zenoh):

| codec | input message | output |
|---|---|---|
| `ros2_image`, `dimos_lcm_image` | `sensor_msgs/Image`: rgb8, bgr8, rgba8, bgra8, mono8, mono16/16UC1 (top 8 bits), or `jpeg`/`png` data in an Image (dimos' jpeg-encoded Image) | H.264 video track (`sub.mediaStream`) |
| `ros2_compressed_image`, `dimos_lcm_compressed_image` | `sensor_msgs/CompressedImage`: jpeg, png, webp, jxl (magic bytes first, `format` string second) | H.264 video track |
| `ros2_depth`, `dimos_lcm_depth` | `sensor_msgs/Image`: 16UC1, 32FC1, mono16 | lossless depth: `msg.decoded` is a depth image (below) |
| `ros2_compressed_depth`, `dimos_lcm_compressed_depth` | `sensor_msgs/CompressedImage`: 16-bit gray png or jxl, ROS `compressedDepth` png (12-byte header skipped; its quantized 32FC1 form is refused) | lossless depth |
| `ros2_pointcloud2`, `dimos_lcm_pointcloud2` | `sensor_msgs/PointCloud2`, any field layout | quantized points: `msg.decoded` is a point cloud (below) |
| `ros2_raw_audio`, `dimos_lcm_raw_audio` | `foxglove_msgs/RawAudio`: `pcm-s16`, any rate (resampled to 48 kHz unless Opus takes it), the first 2 channels | Opus audio track (`sub.mediaStream`) |

Image encodings also go on the data channel (`channel: "data"`), as a file, with `encodeOptions.format`:
`"passthrough"` (compressed images' default: the file as sent), `"jpeg"` (raw images' default; re-encoded at the
allocator's quality, so it shrinks when the link is squeezed, or the file itself when it already is a JPEG and the
quality is 1) or `"png"` (lossless). The page gets the file in `msg.bytes` (`createImageBitmap(new Blob([msg.bytes]))`
gives its pixels). Depth, point clouds and audio take no options.

Every codec reads its message's header (`MessageEncoding::header`: exact `sec`/`nanosec`, `frame_id`, dimos LCM's
`seq`; audio its `timestamp`), and zenoh-gateway 0.5.32+ sends it, with the zenoh sample's encoding and attachment,
alongside whatever the codec made of the message on every channel: `msg.header` (`{ stamp, sec, nanosec, frameId, seq? }`),
`msg.encoding`, `msg.attachment`; `subscribe(key, { metadata: false })` goes without. On video it is also
`msg.video.stamp` / `msg.video.frameId`, and `sub.videoFrameInfo(rtpTimestamp)` for the frame on screen. Point clouds and
depth also keep `stamp` and `frameId` in their fields (0.3.7+), and `encodeOptions.header: true` still wraps a
data-channel image file with its header as a fields message, `msg.decoded` `{stamp, frameId, format, data}` (0.3.8+).

To add a message type: a new file in `src/codecs/msgs/` with its parser and a `zenoh_gateway::MessageEncoding` impl named after
the file, listed in `src/codecs/msgs/mod.rs` and `codecs::all()` (a test checks every file is registered). Shared
pieces are in `src/utils/`: the CDR and LCM readers, image decoders (`jpeg`, `png`, `webp`, `jxl`,
`raw_pixels`, `compressed_image`), `pcm`, `depth` and `pointcloud`.

Inputs:
- ROS 2 over rmw_zenoh: key `<domain>/<topic>/<pkg>::msg::dds_::<Type>_/RIHS01_<hash>`, payload CDR with
  the 4-byte encapsulation header (little or big endian honored).
- dimos over zenoh: key `<topic>/<msg_name>` (e.g. `dimos/camera/color/sensor_msgs.Image`), payload in
  the dimos message format (big endian) with its 8-byte type fingerprint, which the codec checks (a
  wrong type is an error, counted in zenoh-gateway's `codecErrors` / `lastCodecError` stats).
- mono16 is ambiguous (IR intensity or depth-like); the subscriber decides: `*_image` shows its top
  8 bits as gray video, `*_depth` delivers it losslessly with encoding `mono16`.
- Decoders are pure Rust (zune-jpeg, png, image-webp, jxl-oxide), or libjpeg-turbo for JPEG with the
  `turbojpeg` feature (vendored, statically linked; building needs cmake, and nasm on x86; ~2x faster
  on a Jetson Orin, ~1.4x on an Apple M core). A YCbCr JPEG with even sides decodes straight to I420
  (full range → BT.601 limited), never through RGB (with `turbojpeg`, from the JPEG's own planes, no
  chroma upsampling); everything else decodes to RGB8.

Depth stays lossless: quality only lowers resolution, by an integer stride `round(1 / (1/8 + 7/8 q))`
(1 at q = 1, 2 at 0.5, 8 at 0), nearest neighbor (every value is a source value, never a blend).
`data` is a `Uint16Array` (16UC1, mono16) or `Float32Array` (32FC1).

Point clouds: points with a non-finite x, y or z are skipped; fields are read by name (`x`, `y`, `z`,
optional `intensity`) at their offsets with any PointField datatype, honoring `point_step`, `row_step`
and `is_bigendian`. Quality q thins the cloud to a budget of 1 point in `round(1 / q)` (at most 16 at
q = 0), within 10%: the first point of each cell of a voxel grid sized to the budget, in message order,
never moved or merged, in any units; so what is kept is spread over the cloud's space instead of every
Nth point of each scan line (`keepEvery`: source points per sent point, rounded). Coordinates are int16 around a per-message origin: `x = originX + qx × scale`.
Error per axis of a sent point is at most `scale / 2` (`maxError`), where `scale = (largest
bounding-box extent / 2) / 32767` in the cloud's own units, plus f32 rounding. Intensity is scaled to
u8 over the message's min..max (`intensityMin`, `intensityScale`). `positions` is a `Float32Array`.

## Hardware encoders

`zenoh_dimos_codecs::encoders`: hardware H.264 encoders as zenoh-gateway `VideoEncoder`s for
`ServerBuilder::video_encoder` (formerly the zenoh-web-encoders crate). [zenoh-gateway-cli](https://github.com/dimensionalOS/zenoh-gateway-cli)
and [zenoh-gateway-relay](https://github.com/jeff-hykin/zenoh-gateway-relay) use them (`--video-encoder auto|software|videotoolbox|gstreamer`).

| feature | backend | needs |
|---|---|---|
| `videotoolbox` | macOS VideoToolbox (the media engine): constrained baseline, low-latency rate control | macOS (does nothing elsewhere) |
| `gstreamer` | the first GStreamer hardware encoder that works: `nvv4l2h264enc` (Jetson), `nvh264enc` (NVENC), `vah264enc` / `vaapih264enc` (VAAPI) | GStreamer 1.x at runtime only: it is loaded with `dlopen`, so building needs nothing and a machine without it falls back to software |

```toml
zenoh-dimos-codecs = { git = "https://github.com/dimensionalOS/zenoh-dimos-codecs", rev = "<commit>", features = ["videotoolbox", "gstreamer"] }
```

```rust
let selected = zenoh_dimos_codecs::encoders::select(zenoh_dimos_codecs::encoders::Backend::Auto)?;
if let Some(factory) = selected.factory {
    builder = builder.video_encoder(factory); // else zenoh-gateway's software H.264
}
```

- `select` probes each backend by encoding a test frame; `Auto` tries VideoToolbox, then GStreamer, then picks
  software. A named backend that doesn't work is an error.
- Each encoder is wrapped in `Fallback`: after its first error it hands over to openh264 for good (a new software
  encoder starts with a keyframe).
- All of them encode at the bitrate zenoh-gateway grants (changed in place, no keyframe), restart on a new picture size,
  give keyframes on request, and signal zenoh-gateway's colors (BT.601 matrix, BT.709 primaries and transfer).
- `examples/encode_file.rs` encodes raw RGB frames (or a moving test pattern) with any backend to an `.h264` file, to
  try an encoder on a machine and measure it offline.

Measured on the zenoh-gateway bench scene (720p60, offline, decoded by ffmpeg): VideoToolbox 3.95 Mbit/s → 29.34 dB,
8.2 → 29.68, 15.4 → 29.90; openh264 4.0 → 29.37, 8.3 → 29.68, 16.2 → 29.88. Same quality per bit, but on the media
engine instead of a core per stream.

On a Jetson AGX Orin (JetPack 6, GStreamer 1.20; under load from the robot's own stack): `nvv4l2h264enc` took 6-14 ms
per 720p frame (three frames in flight) against openh264's 34 ms, hit its bitrate, and scored 30.27 dB on the bench
scene at 16.6 Mbit/s against openh264's 29.57.

## Wire formats

Both are zenoh-gateway fields messages (zenoh-gateway SPEC "Fields"), zstd-compressed by default
(`Codec::default_compress`; a subscription's `compress: "none"` turns it off). The client decodes them
into `msg.decoded`:

- depth (`version` 2): `{ version, encoding: "16UC1" | "32FC1" | "mono16", stride, width, height,
  sourceWidth, sourceHeight, data }`; `data` is width × height values, row-major.
- point cloud (`version` 3): `{ version, count, sourceCount, origin (Float32Array of 3), scale, keepEvery,
  maxError, positions (Float32Array, x, y, z per point, sent as int16 scaled by origin and scale),
  intensity (Uint8Array, only if the cloud has intensity), intensityMin, intensityScale }`.

## Nix / cross compiling

`nix build .#zenoh-dimos-codecs-example` (native) and `.#zenoh-dimos-codecs-example-aarch64-linux` /
`-x86_64-linux` build `nix_smoke_test/` with both encoder features: `zenoh-dimos-codecs-example [auto|software|videotoolbox|gstreamer] [frames]`
lists the codecs, starts and stops a loopback server with them, then selects an encoder and times it on a 720p test
pattern. Built with zenoh-gateway's `lib.crossRust`: crate2nix, one derivation per crate shared with the other zenoh-gateway
flakes, Linux cross compiled with zig (glibc 2.35). GStreamer is opened at runtime, so the Linux builds need no
GStreamer and the feature is always compiled in. Pass `--max-jobs auto`. After changing `nix_smoke_test/Cargo.lock`,
`nix run github:dimensionalOS/zenoh-gateway#crate2nix -- generate` in `nix_smoke_test/`. To build your own crate that uses this one,
see zenoh-gateway's README "Nix / cross compiling".

## Tests

```sh
cargo test && cargo clippy --all-targets   # parsers, decoders, encoders against test/fixtures
```

`test/fixtures/` holds one payload per message (made by dimos's own encoder and by rosbags;
`generate.py` rebuilds them, `manifest.json` describes them). zenoh-gateway-cli's `test/codecs.js` sends
every fixture through every codec into headless Chrome.
