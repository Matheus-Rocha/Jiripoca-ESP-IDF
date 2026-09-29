"""Converts a camN.mjp file recorded by the Jiripoca S3 into JPEG frames, a CSV index and an MP4.

Usage: python cam_decode.py cam12.mjp [output_dir]

The MP4 needs ffmpeg on PATH; frame timing comes from the capture timestamps, so the video
keeps real time even if the link dropped frames. capture_ms uses the same clock as the
flight log "time" field (ms since S3 boot), which is how the video is synced with telemetry.
"""

import csv
import os
import shutil
import struct
import subprocess
import sys

FILE_HDR = struct.Struct("<IHHI")  # magic, version, reserved, s3_open_ms
REC_HDR = struct.Struct("<IIIII B3x")  # magic, seq, capture_ms, write_ms, len, status
FILE_MAGIC = 0x5649434A
REC_MAGIC = 0x4D52464A

STATUS_BITS = ["INITIALIZED", "ARMED", "BOOST", "COAST", "DROGUE", "MAIN", "LANDING", "LANDED"]


def read_records(data):
    pos = FILE_HDR.size
    while pos + REC_HDR.size <= len(data):
        magic, seq, capture_ms, write_ms, length, status = REC_HDR.unpack_from(data, pos)
        if magic != REC_MAGIC:
            # Resynchronize after a damaged record
            nxt = data.find(struct.pack("<I", REC_MAGIC), pos + 1)
            if nxt < 0:
                return
            pos = nxt
            continue
        start = pos + REC_HDR.size
        jpeg = data[start:start + length]
        if len(jpeg) < length:
            print(f"Truncated last frame {seq} (power loss or impact), ignored")
            return
        yield seq, capture_ms, write_ms, status, jpeg
        pos = start + length


def status_names(status):
    return "|".join(name for bit, name in enumerate(STATUS_BITS) if status & (1 << bit))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    src = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + "_frames"
    os.makedirs(out_dir, exist_ok=True)

    with open(src, "rb") as f:
        data = f.read()

    magic, version, _, open_ms = FILE_HDR.unpack_from(data, 0)
    if magic != FILE_MAGIC:
        sys.exit(f"{src} is not a Jiripoca camera file")
    print(f"File version {version}, opened at {open_ms} ms after S3 boot")

    frames = []
    with open(os.path.join(out_dir, "index.csv"), "w", newline="") as idx:
        writer = csv.writer(idx)
        writer.writerow(["file", "seq", "capture_ms", "write_ms", "bytes", "status"])
        for seq, capture_ms, write_ms, status, jpeg in read_records(data):
            name = f"frame_{len(frames):06d}.jpg"
            with open(os.path.join(out_dir, name), "wb") as jf:
                jf.write(jpeg)
            writer.writerow([name, seq, capture_ms, write_ms, len(jpeg), status_names(status)])
            frames.append((name, capture_ms))

    if not frames:
        sys.exit("No frames found")

    span_s = (frames[-1][1] - frames[0][1]) / 1000
    fps = (len(frames) - 1) / span_s if span_s > 0 else 0
    print(f"{len(frames)} frames, {span_s:.1f} s, average {fps:.1f} fps -> {out_dir}")

    if not shutil.which("ffmpeg"):
        print("ffmpeg not found, skipping MP4 (frames and index.csv are ready)")
        return

    list_path = os.path.join(out_dir, "frames.txt")
    with open(list_path, "w") as lst:
        for (name, t), (_, t_next) in zip(frames, frames[1:] + [(None, frames[-1][1] + 40)]):
            lst.write(f"file '{name}'\nduration {max(t_next - t, 1) / 1000:.3f}\n")
        lst.write(f"file '{frames[-1][0]}'\n")

    mp4 = os.path.join(out_dir, "video.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", list_path,
                    "-fps_mode", "vfr", "-pix_fmt", "yuv420p", mp4], check=True)
    print(f"Video: {mp4}")


if __name__ == "__main__":
    main()
