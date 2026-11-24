"""Stage 3 (dimos venv + ffmpeg with libx264 and libx265): foxglove_msgs/CompressedVideo, one keyframe of the pattern each.

Writes compressed/pattern.{h264,h265} (Annex-B access units: parameter sets, then the IDR), dimos/compressed_video_*.bin
(dimos's own CompressedVideo.lcm_encode) and ros2/compressed_video_h265.cdr (CDR little endian, packed here: rosbags
has no foxglove_msgs). compressed/pattern_bare.h265 is the IDR without its parameter sets (as an RTSP relay that drops the out-of-band ones sends it).
"""

import json
from pathlib import Path
import struct
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import common as c  # noqa: E402

from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo  # noqa: E402

ENCODERS = {
    "h264": ["-c:v", "libx264", "-profile:v", "high", "-x264-params", "keyint=1", "-f", "h264"],
    "h265": ["-c:v", "libx265", "-x265-params", "keyint=1:repeat-headers=1:log-level=error", "-f", "hevc"],
}


def encode(fmt):
    raw = c.rgb_pattern().tobytes()
    command = ["ffmpeg", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{c.WIDTH}x{c.HEIGHT}", "-i", "-", "-frames:v", "1", "-pix_fmt", "yuv420p", *ENCODERS[fmt], "-"]
    return subprocess.run(command, input=raw, capture_output=True, check=True).stdout


def cdr_string(body, text):
    encoded = text.encode() + b"\0"
    body += b"\0" * (-len(body) % 4) + struct.pack("<I", len(encoded)) + encoded
    return body


def ros2_payload(data, fmt):
    sec, nanosec = int(c.TS), round((c.TS - int(c.TS)) * 1e9)
    body = struct.pack("<II", sec, nanosec)
    body = cdr_string(body, c.COLOR_FRAME)
    body += b"\0" * (-len(body) % 4) + struct.pack("<I", len(data)) + data
    body = cdr_string(body, fmt)
    return b"\x00\x01\x00\x00" + body


PATTERN_RGB = {"top_left": [255, 0, 0], "top_right": [0, 255, 0], "bottom_left": [0, 0, 255], "bottom_right": [255, 255, 255]}
entries = []

for fmt in ENCODERS:
    data = encode(fmt)
    (HERE / "compressed" / f"pattern.{fmt}").write_bytes(data)
    message = CompressedVideo(data, format=fmt, frame_id=c.COLOR_FRAME, ts=c.TS)
    payload = message.lcm_encode()
    back = CompressedVideo.lcm_decode(payload)
    assert back.data.tobytes() == data and back.format == fmt and back.frame_id == c.COLOR_FRAME
    (HERE / "dimos" / f"compressed_video_{fmt}.bin").write_bytes(payload)
    entries.append({
        "file": f"dimos/compressed_video_{fmt}.bin", "protocol": "dimos", "bytes": len(payload), "format": fmt, "msg_type": CompressedVideo.msg_name,
        "zenoh_key": f"dimos/camera/color/video/{CompressedVideo.msg_name}", "frame_id": c.COLOR_FRAME, "width": c.WIDTH, "height": c.HEIGHT,
        "encoder": f"ffmpeg {ENCODERS[fmt][1]}, one keyframe with its parameter sets", "compressed_bytes_file": f"compressed/pattern.{fmt}",
        "expected": {"lossless": False, "nominal_quadrant_rgb": PATTERN_RGB},
    })

h265 = (HERE / "compressed" / "pattern.h265").read_bytes()
(HERE / "ros2" / "compressed_video_h265.cdr").write_bytes(ros2_payload(h265, "h265"))
# the IDR slice alone (NAL type 19/20), from its start code
idr = next(at for at in range(len(h265) - 4) if h265[at:at + 3] == b"\0\0\1" and (h265[at + 3] >> 1) & 0x3F in (19, 20))
(HERE / "compressed" / "pattern_bare.h265").write_bytes(b"\0" + h265[idr:])

# (generate.py folds it into manifest.json; the ROS 2 fixture has no RIHS01 key, so only its parser is tested)
(HERE / "manifest.video.json").write_text(json.dumps(entries, indent=2) + "\n")
