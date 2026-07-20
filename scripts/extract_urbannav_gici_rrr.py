#!/usr/bin/env python3
"""Regenerate GICI file-mode RRR inputs (imu.bin.txt + camera.bin) from the
UrbanNav raw ROS 1 sensors.bag.

The "re-prepared" UrbanNav layout dropped the derived gici_rrr/ folder that the
file-mode baseline (scripts/run_urbannav_rrr_baseline.py) depends on. This script
rebuilds it from the canonical source (the sensors.bag) with the exact byte
formats GICI's file readers expect:

  * gici_rrr/imu.bin.txt  -- imu-text: header line (skipped, contains "Timestamp")
      then one row per sample "t ax ay az gx gy gz" (space-separated).
      Time is UTC-unix seconds = the IMU header stamp = the value stored in
      _standardization/imu_xsens_standardized.csv (verified equal). GICI's
      ImuTextReader sscanf order is (time, acc[0..2], gyro[0..2]).

  * gici_rrr/camera.bin   -- GICI image-pack (src/stream/format_image.c), one
      frame per /zed2/camera/left/image_raw message, grayscale 672x376 step=1.
      Grayscale replicates GICI's ros_stream.cpp cv_bridge::toCvCopy(MONO8) i.e.
      cv2.cvtColor(BGR, GRAY). Frame time = image header stamp (UTC-unix), the
      same convention GICI stores (verified against the old extractor).

IMU defaults to the standardized CSV (exact, fast, no 68 GB bag scan); pass
--imu-source bag to read /imu/data directly instead.

Usage:
  python3 scripts/extract_urbannav_gici_rrr.py deep   [--root PATH] [--force]
  python3 scripts/extract_urbannav_gici_rrr.py medium [--root PATH] [--force]
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# --- image-pack constants (src/stream/format_image.c) ---
IMG_PREAMB = bytes([0xFE, 0xCA, 0x00, 0xFF, 0x00, 0xFF])
IMG_TAIL = bytes([0xCC, 0xFE, 0xFF, 0x00, 0xFF, 0x00])

DEFAULT_ROOT = Path("/media/theph/Data1/Research/dataset/UrbanNav")


@dataclass(frozen=True)
class Scene:
    key: str
    dirname: str
    sensors_bag: str
    imu_csv: str
    cam_w: int = 672
    cam_h: int = 376
    cam_step: int = 1
    imu_topic: str = "/imu/data"
    cam_topic: str = "/zed2/camera/left/image_raw"


SCENES: dict[str, Scene] = {
    "deep": Scene(
        key="deep",
        dirname="UrbanNav-HK-Deep-Urban-1",
        sensors_bag="ros/UrbanNav-HK_Whampoa-20210521_sensors.bag",
        imu_csv="_standardization/imu_xsens_standardized.csv",
    ),
    "medium": Scene(
        key="medium",
        dirname="UrbanNav-HK-Medium-Urban-1",
        sensors_bag="ros/UrbanNav-HK_TST-20210517_sensors.bag",
        imu_csv="_standardization/imu_xsens_standardized.csv",
    ),
    "harsh": Scene(
        key="harsh",
        dirname="UrbanNav-HK-Harsh-Urban-1",
        sensors_bag="ros/UrbanNav-HK_Mongkok-20210518_sensors.bag",
        imu_csv="_standardization/imu_xsens_standardized.csv",
    ),
}


def _encode_image_frame(t_sec: int, t_nanosec: int, gray: np.ndarray,
                        width: int, height: int, step: int) -> bytes:
    """Encode one GICI image-pack frame. Layout (all multi-byte big-endian):
       preamble[6] | len24[3] | time.time u32 | nanosec u32 | w u16 | h u16 |
       step u8 | pixels[w*h*step] | tail[6]
       len24 = total - 9 = 19 + w*h*step  (matches format_image.c gen_img)."""
    pixels = gray.tobytes()
    assert len(pixels) == width * height * step, (len(pixels), width * height * step)
    body = (
        struct.pack(">I", t_sec)
        + struct.pack(">I", t_nanosec)
        + struct.pack(">H", width)
        + struct.pack(">H", height)
        + struct.pack("B", step)
        + pixels
    )
    length = len(body) + len(IMG_TAIL)  # 13 + w*h*step + 6 = 19 + w*h*step
    return IMG_PREAMB + length.to_bytes(3, "big") + body + IMG_TAIL


def extract_camera(bag: Path, out_bin: Path, scene: Scene, limit: int | None) -> dict:
    import cv2
    from rosbags.highlevel import AnyReader

    w, h, step = scene.cam_w, scene.cam_h, scene.cam_step
    n = 0
    first_t = last_t = None
    non_monotonic = 0
    tmp = out_bin.with_suffix(".bin.partial")
    with AnyReader([bag]) as reader, tmp.open("wb") as fp:
        conns = [c for c in reader.connections if c.topic == scene.cam_topic]
        if not conns:
            raise RuntimeError(f"topic {scene.cam_topic} not found in {bag}")
        for con, _ts, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, con.msgtype)
            if m.width != w or m.height != h:
                raise RuntimeError(
                    f"frame {n}: {m.width}x{m.height} != expected {w}x{h}")
            buf = np.frombuffer(m.data, dtype=np.uint8)
            enc = m.encoding.lower()
            if enc == "bgr8":
                img = buf.reshape(h, m.step)[:, : w * 3].reshape(h, w, 3)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            elif enc == "rgb8":
                img = buf.reshape(h, m.step)[:, : w * 3].reshape(h, w, 3)
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            elif enc in ("mono8", "8uc1"):
                gray = buf.reshape(h, m.step)[:, :w].copy()
            else:
                raise RuntimeError(f"unsupported image encoding {m.encoding!r}")
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if last_t is not None and t < last_t:
                non_monotonic += 1
            first_t = t if first_t is None else first_t
            last_t = t
            fp.write(_encode_image_frame(
                m.header.stamp.sec, m.header.stamp.nanosec, gray, w, h, step))
            n += 1
            if n % 2000 == 0:
                print(f"  [camera] {n} frames  t={t:.3f}", flush=True)
            if limit and n >= limit:
                break
    tmp.replace(out_bin)
    frame_bytes = 28 + w * h * step
    return {
        "frames": n,
        "bytes": out_bin.stat().st_size,
        "frame_bytes": frame_bytes,
        "size_ok": out_bin.stat().st_size == n * frame_bytes,
        "first_t": first_t,
        "last_t": last_t,
        "span_s": (last_t - first_t) if (first_t and last_t) else 0.0,
        "rate_hz": (n / (last_t - first_t)) if (first_t and last_t and last_t > first_t) else 0.0,
        "non_monotonic": non_monotonic,
    }


def _imu_rows_from_csv(csv_path: Path):
    """Yield (t, ax, ay, az, gx, gy, gz) from standardized IMU CSV.
    CSV order: time_unix_s, gyro_x/y/z, acc_x/y/z, quat... -> reorder to acc,gyro."""
    with csv_path.open() as fp:
        header = fp.readline()  # skip CSV header
        assert header.startswith("time_unix_s"), header[:40]
        for line in fp:
            p = line.split(",")
            t = p[0]
            gx, gy, gz = p[1], p[2], p[3]
            ax, ay, az = p[4], p[5], p[6]
            yield t, ax, ay, az, gx, gy, gz


def _imu_rows_from_bag(bag: Path, scene: Scene):
    from rosbags.highlevel import AnyReader

    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == scene.imu_topic]
        if not conns:
            raise RuntimeError(f"topic {scene.imu_topic} not found in {bag}")
        for con, _ts, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, con.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            a, g = m.linear_acceleration, m.angular_velocity
            yield (f"{t:.9f}", repr(a.x), repr(a.y), repr(a.z),
                   repr(g.x), repr(g.y), repr(g.z))


def extract_imu(scene: Scene, root: Path, out_txt: Path, source: str) -> dict:
    tmp = out_txt.with_suffix(".txt.partial")
    n = 0
    first_t = last_t = None
    if source == "csv":
        rows = _imu_rows_from_csv(root / scene.imu_csv)
    else:
        rows = _imu_rows_from_bag(root / scene.sensors_bag, scene)
    with tmp.open("w") as fp:
        # Header line contains "Timestamp" so GICI's ImuTextReader skips it.
        fp.write("Timestamp acc_x acc_y acc_z gyro_x gyro_y gyro_z\n")
        for t, ax, ay, az, gx, gy, gz in rows:
            fp.write(f"{t} {ax} {ay} {az} {gx} {gy} {gz}\n")
            ft = float(t)
            first_t = ft if first_t is None else first_t
            last_t = ft
            n += 1
    tmp.replace(out_txt)
    return {
        "samples": n,
        "source": source,
        "first_t": first_t,
        "last_t": last_t,
        "span_s": (last_t - first_t) if (first_t and last_t) else 0.0,
        "rate_hz": (n / (last_t - first_t)) if (first_t and last_t and last_t > first_t) else 0.0,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene", choices=sorted(SCENES))
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT,
                    help="UrbanNav parent dir (contains UrbanNav-HK-*-1/)")
    ap.add_argument("--imu-source", choices=["csv", "bag"], default="csv")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--skip-imu", action="store_true")
    ap.add_argument("--skip-camera", action="store_true")
    ap.add_argument("--limit", type=int, default=None,
                    help="max camera frames (smoke test)")
    args = ap.parse_args()

    scene = SCENES[args.scene]
    root = args.root
    if root.name != scene.dirname:
        root = root / scene.dirname
    if not root.is_dir():
        print(f"ERROR: dataset root missing: {root}", file=sys.stderr)
        return 1

    gici = root / "gici_rrr"
    gici.mkdir(parents=True, exist_ok=True)
    imu_out = gici / "imu.bin.txt"
    cam_out = gici / "camera.bin"

    report: dict = {"scene": scene.key, "root": str(root)}

    if not args.skip_imu:
        if imu_out.is_file() and not args.force:
            print(f"SKIP imu (exists): {imu_out}  (use --force)")
        else:
            print(f"[imu] source={args.imu_source} -> {imu_out}", flush=True)
            report["imu"] = extract_imu(scene, root, imu_out, args.imu_source)
            print(f"  imu: {report['imu']['samples']} samples "
                  f"@{report['imu']['rate_hz']:.1f} Hz span={report['imu']['span_s']:.1f}s")

    if not args.skip_camera:
        if cam_out.is_file() and not args.force and not args.limit:
            print(f"SKIP camera (exists): {cam_out}  (use --force)")
        else:
            bag = root / scene.sensors_bag
            if not bag.is_file():
                print(f"ERROR: sensors bag missing: {bag}", file=sys.stderr)
                return 1
            print(f"[camera] {bag} -> {cam_out}", flush=True)
            report["camera"] = extract_camera(bag, cam_out, scene, args.limit)
            c = report["camera"]
            print(f"  camera: {c['frames']} frames @{c['rate_hz']:.2f} Hz "
                  f"span={c['span_s']:.1f}s size_ok={c['size_ok']} "
                  f"non_monotonic={c['non_monotonic']}")

    import json
    (gici / "extract_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nOK: wrote gici_rrr under {root}")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
