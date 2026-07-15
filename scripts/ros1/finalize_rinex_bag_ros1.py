#!/usr/bin/env python3
"""Merge republished RINEX GNSS (ROS 1 bag) + native sensors into one ROS 1 bag.

Same UTC reclock / burst semantics as scripts/ros2/finalize_rinex_bag.py, but
output is a single .bag for gici_ros_main + rosbag play (no ROS 2).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader, Writer

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "ros2"))
import urbannav_rrr_to_ros2 as u  # noqa: E402

IMU_TOPIC = u.IMU_TOPIC
CAM_TOPIC = u.CAM_TOPIC
OBS_SUFFIX = "/observations"
ROVER_TOPIC = "/gici/gnss_rover/observations"
BURST_SUFFIXES = (
    "/ephemerides",
    "/ionosphere_parameter",
    "/antenna_position",
    "/code_bias",
)
EPH_BAGS = (
    "gnss_ephemeris_G.bag",
    "gnss_ephemeris_R.bag",
    "gnss_ephemeris_E.bag",
    "gnss_ephemeris_C.bag",
)


def sha256_file(path: Path) -> str:
    if not path.is_file():
        return "MISSING"
    h = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_commit() -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(REPO), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=True)
        dirty = subprocess.run(
            ["git", "-C", str(REPO), "status", "--porcelain"],
            capture_output=True, text=True, check=True).stdout.strip()
        return out.stdout.strip() + ("-dirty" if dirty else "")
    except Exception:
        return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--republish-bag", required=True, help="ROS 1 republish .bag")
    ap.add_argument("--sensors", required=True, help="UrbanNav ..._sensors.bag")
    ap.add_argument("--out", required=True, help="Output merged .bag (must not exist)")
    ap.add_argument("--rover", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--eph", required=True)
    ap.add_argument("--dcb", required=True)
    ap.add_argument("--leap", type=float, default=18.0)
    ap.add_argument("--sensor-margin", type=float, default=5.0)
    ap.add_argument("--ref-margin", type=float, default=120.0)
    ap.add_argument("--eph-dir", default="", help="Native GNSS ephemeris bags directory")
    ap.add_argument("--window", type=float, default=0.0,
                    help="Optional max drive length in seconds from t0 (0 = no extra cap).")
    args = ap.parse_args()

    republish_bag = Path(args.republish_bag)
    sensors = Path(args.sensors)
    out_bag = Path(args.out)
    if out_bag.exists():
        print(f"ERROR: {out_bag} exists; remove first.", file=sys.stderr)
        return 1
    for p in (republish_bag, sensors):
        if not p.exists():
            print(f"ERROR: missing {p}", file=sys.stderr)
            return 1

    ts1 = u.build_ros1_typestore()

    rover_records: list[tuple[float, str, str, object]] = []
    ref_records: list[tuple[float, str, str, object]] = []
    burst_records: list[tuple[str, str, object]] = []

    with Reader(republish_bag) as r:
        for c, _t, raw in r.messages():
            topic = c.topic
            msg = ts1.deserialize_ros1(raw, c.msgtype)
            if topic.endswith(OBS_SUFFIX):
                if not msg.observations:
                    continue
                utc = u.gpst_from_obs(msg) - args.leap
                if topic == ROVER_TOPIC:
                    rover_records.append((utc, topic, c.msgtype, msg))
                else:
                    ref_records.append((utc, topic, c.msgtype, msg))
            elif topic.endswith(BURST_SUFFIXES):
                burst_records.append((topic, c.msgtype, msg))

    eph_dir = Path(args.eph_dir) if args.eph_dir else None
    if eph_dir and eph_dir.is_dir():
        for bag_name in EPH_BAGS:
            bag_path = eph_dir / bag_name
            if not bag_path.is_file():
                continue
            with Reader(bag_path) as r:
                for c, _t, raw in r.messages():
                    if not c.topic.endswith(BURST_SUFFIXES):
                        continue
                    msg = ts1.deserialize_ros1(raw, c.msgtype)
                    burst_records.append((c.topic, c.msgtype, msg))

    if not rover_records:
        print("ERROR: no rover observations in republish bag.", file=sys.stderr)
        return 1

    rover_records.sort(key=lambda x: x[0])
    rover_lo, rover_hi = rover_records[0][0], rover_records[-1][0]
    ref_lo, ref_hi = rover_lo - args.ref_margin, rover_hi + args.ref_margin
    ref_kept = [rr for rr in ref_records if ref_lo <= rr[0] <= ref_hi]
    obs_records = sorted(rover_records + ref_kept, key=lambda x: x[0])
    obs_lo = obs_records[0][0]

    def first_sensor_utc(topic: str):
        with Reader(sensors) as r:
            for c, _t, raw in r.messages():
                if c.topic != topic:
                    continue
                src = ts1.deserialize_ros1(raw, c.msgtype)
                return src.header.stamp.sec + src.header.stamp.nanosec * 1e-9
        return None

    imu_t0 = first_sensor_utc(IMU_TOPIC)
    cam_t0 = first_sensor_utc(CAM_TOPIC)
    sensor_starts = [v for v in (imu_t0, cam_t0) if v is not None and v >= obs_lo - args.sensor_margin]
    t0 = min([obs_lo] + sensor_starts)
    sensor_lo = t0 - args.sensor_margin
    sensor_hi = rover_hi + args.sensor_margin
    if args.window > 0:
        sensor_hi = min(sensor_hi, t0 + args.window)
        obs_records = [o for o in obs_records if o[0] <= t0 + args.window]

    print(f"t0(utc)={t0:.3f}  rover=[{rover_lo:.3f},{rover_hi:.3f}]  "
          f"ref_kept={len(ref_kept)}  sensor=[{sensor_lo:.3f},{sensor_hi:.3f}]")

    burst_base_ns = int((t0 - 2.0) * 1e9)
    counts: dict[str, int] = {}
    n_drop = 0
    out_bag.parent.mkdir(parents=True, exist_ok=True)

    with Writer(out_bag) as writer:
        conns: dict[str, object] = {}

        def write_msg(topic: str, rtype: str, wt_ns: int, msg) -> None:
            data = ts1.serialize_ros1(msg, rtype)
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, rtype, typestore=ts1)
            writer.write(conns[topic], wt_ns, data)
            counts[topic] = counts.get(topic, 0) + 1

        for i, (topic, rtype, msg) in enumerate(burst_records):
            write_msg(topic, rtype, burst_base_ns + i * 1_000_000, msg)

        for utc, topic, rtype, msg in obs_records:
            write_msg(topic, rtype, int(utc * 1e9), msg)

        with Reader(sensors) as r:
            for c, _t, raw in r.messages():
                if c.topic not in (IMU_TOPIC, CAM_TOPIC):
                    continue
                msg = ts1.deserialize_ros1(raw, c.msgtype)
                utc = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if utc < sensor_lo or utc > sensor_hi:
                    n_drop += 1
                    continue
                write_msg(c.topic, c.msgtype, int(utc * 1e9), msg)

    manifest = {
        "converter": "scripts/ros1/finalize_rinex_bag_ros1.py",
        "git_commit": git_commit(),
        "leap_seconds": args.leap,
        "t0_utc": t0,
        "inputs": {
            "rover": {"path": str(args.rover), "sha256": sha256_file(Path(args.rover))},
            "base": {"path": str(args.base), "sha256": sha256_file(Path(args.base))},
            "eph": {"path": str(args.eph), "sha256": sha256_file(Path(args.eph))},
            "dcb": {"path": str(args.dcb), "sha256": sha256_file(Path(args.dcb))},
            "sensors": {"path": str(sensors), "sha256": sha256_file(sensors)},
        },
        "topics": counts,
        "sensors_dropped_outside_window": n_drop,
        "rover_drive_utc": [rover_lo, rover_hi],
    }
    manifest_path = out_bag.with_suffix(".manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2))

    print(f"Done -> {out_bag}")
    for topic in sorted(counts):
        print(f"  {topic:45s} {counts[topic]:8d}")
    print(f"  manifest -> {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
