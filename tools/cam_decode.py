"""Converts a camN.vid file recorded by the Jiripoca camera into an H.264 stream, a CSV index and an MP4.

Usage: python cam_decode.py cam12.vid [output_dir]

The MP4 needs ffmpeg on PATH and is made without re-encoding. capture_ms in index.csv uses the same
clock as the flight log "time" field (ms since boot), which is how the video is synced with telemetry.
"""

import csv
import os
import shutil
import struct
import subprocess
import sys

FILE_HDR = struct.Struct("<IHHI")  # magic, version, fps, open_ms
REC_HDR = struct.Struct("<IIIIIBB2x")  # magic, seq, capture_ms, write_ms, len, status, keyframe
FILE_MAGIC = 0x5649434A
REC_MAGIC = 0x4D52464A
FILE_VERSION = 2

STATUS_BITS = ["INITIALIZED", "ARMED", "BOOST", "COAST", "DROGUE", "MAIN", "LANDING", "LANDED"]


def read_records(data):
    pos = FILE_HDR.size
    while pos + REC_HDR.size <= len(data):
        magic, seq, capture_ms, write_ms, length, status, keyframe = REC_HDR.unpack_from(data, pos)
        if magic != REC_MAGIC:
            # Resynchronize after a damaged record
            nxt = data.find(struct.pack("<I", REC_MAGIC), pos + 1)
            if nxt < 0:
                return
            pos = nxt
            continue
        start = pos + REC_HDR.size
        frame = data[start:start + length]
        if len(frame) < length:
            print(f"Truncated last frame {seq} (power loss or impact), ignored")
            return
        yield seq, capture_ms, write_ms, status, bool(keyframe), frame
        pos = start + length


def status_names(status):
    return "|".join(name for bit, name in enumerate(STATUS_BITS) if status & (1 << bit))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    src = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + "_video"
    os.makedirs(out_dir, exist_ok=True)

    with open(src, "rb") as f:
        data = f.read()

    magic, version, fps, open_ms = FILE_HDR.unpack_from(data, 0)
    if magic != FILE_MAGIC:
        sys.exit(f"{src} is not a Jiripoca camera file")
    if version != FILE_VERSION:
        sys.exit(f"File version {version} not supported (this tool reads version {FILE_VERSION}, H.264)")
    print(f"H.264 file, sensor {fps} fps, opened at {open_ms} ms after boot")

    h264_path = os.path.join(out_dir, "video.h264")
    times = []
    keyframes = 0
    with open(h264_path, "wb") as h264, open(os.path.join(out_dir, "index.csv"), "w", newline="") as idx:
        writer = csv.writer(idx)
        writer.writerow(["frame", "seq", "capture_ms", "write_ms", "bytes", "keyframe", "status"])
        for seq, capture_ms, write_ms, status, keyframe, frame in read_records(data):
            if not times and not keyframe:
                print("Warning: stream does not start on a keyframe, the first frames will not decode")
            h264.write(frame)
            writer.writerow([len(times), seq, capture_ms, write_ms, len(frame), int(keyframe), status_names(status)])
            times.append(capture_ms)
            keyframes += keyframe

    if not times:
        sys.exit("No frames found")

    span_s = (times[-1] - times[0]) / 1000
    real_fps = (len(times) - 1) / span_s if span_s > 0 else fps
    print(f"{len(times)} frames ({keyframes} keyframes), {span_s:.1f} s, average {real_fps:.1f} fps -> {out_dir}")

    if not shutil.which("ffmpeg"):
        print("ffmpeg not found, skipping MP4 (video.h264 plays in VLC; index.csv has the timestamps)")
        return

    mp4 = os.path.join(out_dir, "video.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{real_fps:.3f}", "-i", h264_path,
                    "-c", "copy", mp4], check=True)
    print(f"Video: {mp4}")


if __name__ == "__main__":
    main()
