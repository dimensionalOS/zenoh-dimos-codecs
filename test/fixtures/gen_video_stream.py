"""ffmpeg with libx264 and libx265: compressed/scroll.{h264,h265}, a stream with predicted frames for stream decoding.

FRAMES pictures of the pattern rolled SHIFT more columns right each, a keyframe every KEYINT (parameter sets in front of
each), an access unit delimiter in front of every access unit (so a test can split them), no B-frames.
"""

from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import common as c  # noqa: E402
import numpy as np  # noqa: E402

FRAMES, KEYINT, SHIFT = 8, 4, 16
ENCODERS = {
    "h264": ["-c:v", "libx264", "-profile:v", "high", "-bf", "0", "-x264-params", f"keyint={KEYINT}:min-keyint={KEYINT}:scenecut=0:aud=1", "-f", "h264"],
    "h265": ["-c:v", "libx265", "-x265-params", f"keyint={KEYINT}:min-keyint={KEYINT}:scenecut=0:bframes=0:repeat-headers=1:aud=1:log-level=error", "-f", "hevc"],
}


def main():
    raw = b"".join(np.roll(c.rgb_pattern(), SHIFT * index, axis=1).tobytes() for index in range(FRAMES))
    for fmt, encoder in ENCODERS.items():
        command = ["ffmpeg", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{c.WIDTH}x{c.HEIGHT}", "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", *encoder, "-"]
        (HERE / "compressed" / f"scroll.{fmt}").write_bytes(subprocess.run(command, input=raw, capture_output=True, check=True).stdout)


if __name__ == "__main__":
    main()
