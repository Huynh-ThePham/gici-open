#!/usr/bin/env python3
"""Finalize a CANONICAL UrbanNav RRR ROS 2 bag from republished RINEX GNSS.

This is Phase 2 of the faithful ROS 2 RRR pipeline. It takes:

  * the *republish* ROS 2 bag (topics under /gici/*), produced by running
    ros2_wrapper/.../config/urbannav_rinex_republish.yaml through gici_ros2_main
    while `ros2 bag record` captures the output. Those messages carry the EXACT
    GNSS data the file-mode RRR baseline consumes (rover .obs, HKKT base .rnx,
    brdc eph .rnx, DCB .BSX), decoded by the core's own RINEX/DCB formators, and
  * the UrbanNav sensors ROS 1 bag (/imu/data + /zed2/camera/left/image_raw).

It writes ONE merged ROS 2 bag whose *record* time equals GICI's internal UTC
measurement time, exactly like scripts/ros2/urbannav_rrr_to_ros2.py:

  * GNSS observations -> record time = gpst(week,tow) - leap  (UTC)
  * ephemeris / ionosphere / antenna / DCB -> bursted at t0 - 2 s (like
    burst_load in file mode; the estimator reads the fields, not the stamp)
  * IMU / camera -> header stamp as-is (already UTC; NOT shifted); bgr8 -> mono8

It also emits manifest.json (SHA256 of every canonical input, converter git
commit, and per-topic message counts + record-time span) so each generated bag
is reproducible and auditable for the thesis.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
from rosbags.rosbag2 import Writer

# Reuse the proven converter helpers (typestores, message convert, bgr8->mono8,
# gpst_from_obs) from the existing merged converter so nothing diverges.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import urbannav_rrr_to_ros2 as u  # noqa: E402

REPO = Path(__file__).resolve().parents[2]

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


def first_sensor_utc(sensors: Path, ts1, topic: str):
    with Reader1(sensors) as r:
        for c, t, raw in r.messages():
            if c.topic != topic:
                continue
            src = ts1.deserialize_ros1(raw, c.msgtype)
            return src.header.stamp.sec + src.header.stamp.nanosec * 1e-9
    return None


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--republish-bag", required=True,
                    help="ROS 2 bag directory recorded from the republish node")
    ap.add_argument("--sensors", required=True, help="UrbanNav ..._sensors.bag (ROS 1)")
    ap.add_argument("--out", required=True,
                    help="Output canonical ROS 2 bag directory (must not exist)")
    ap.add_argument("--rover", required=True, help="Canonical rover RINEX (for manifest)")
    ap.add_argument("--base", required=True, help="Canonical base RINEX (for manifest)")
    ap.add_argument("--eph", required=True, help="Canonical eph RINEX (for manifest)")
    ap.add_argument("--dcb", required=True, help="Canonical DCB BSX (for manifest)")
    ap.add_argument("--leap", type=float, default=18.0,
                    help="GPST-UTC leap seconds (default 18 for 2021). GNSS obs record "
                         "time = gpst(week,tow) - leap. Sensor stamps are NOT shifted.")
    ap.add_argument("--sensor-margin", type=float, default=5.0,
                    help="Keep sensor messages within [t0 - margin, rover_hi + margin].")
    ap.add_argument("--ref-margin", type=float, default=120.0,
                    help="Keep base-station epochs within [rover_lo - margin, rover_hi + margin]. "
                         "The base RINEX is a full day; this clips it to the drive so the bag "
                         "plays in minutes without changing the epochs the estimator actually uses.")
    args = ap.parse_args()

    republish_bag = Path(args.republish_bag)
    sensors = Path(args.sensors)
    out_dir = Path(args.out)
    if out_dir.exists():
        print(f"ERROR: output {out_dir} already exists; remove it first.", file=sys.stderr)
        return 1
    for p in (republish_bag, sensors):
        if not p.exists():
            print(f"ERROR: missing input {p}", file=sys.stderr)
            return 1

    ts2 = u.build_ros2_typestore()
    ts1 = u.build_ros1_typestore()

    # --- Pass 1: read republished GNSS, split obs (reclock) vs burst set ---
    obs_records: list[tuple[float, str, str, bytes]] = []  # (utc, topic, rtype, raw)
    burst_records: list[tuple[str, str, bytes]] = []       # (topic, rtype, raw), in-order
    rover_records: list[tuple[float, str, str, bytes]] = []
    ref_records: list[tuple[float, str, str, bytes]] = []
    with Reader2(republish_bag) as r:
        for c, t, raw in r.messages():
            topic = c.topic
            if topic.endswith(OBS_SUFFIX):
                msg = ts2.deserialize_cdr(raw, c.msgtype)
                if not msg.observations:
                    continue
                utc = u.gpst_from_obs(msg) - args.leap
                if topic == ROVER_TOPIC:
                    rover_records.append((utc, topic, c.msgtype, raw))
                else:
                    ref_records.append((utc, topic, c.msgtype, raw))
            elif topic.endswith(BURST_SUFFIXES):
                burst_records.append((topic, c.msgtype, raw))

    if not rover_records:
        print("ERROR: no rover GNSS observations found in republish bag.", file=sys.stderr)
        return 1

    rover_records.sort(key=lambda x: x[0])
    rover_lo = rover_records[0][0]
    rover_hi = rover_records[-1][0]

    # The base station RINEX is a full-day file (e.g. HKKT 30 s, 2880 epochs over
    # 24 h). Keep only reference epochs bracketing the rover drive: numerically
    # safe (the estimator only uses base obs near rover epochs) and it keeps the
    # bag ~drive-length so playback is minutes, not a full day.
    ref_lo = rover_lo - args.ref_margin
    ref_hi = rover_hi + args.ref_margin
    ref_kept = [rr for rr in ref_records if ref_lo <= rr[0] <= ref_hi]
    n_ref_dropped = len(ref_records) - len(ref_kept)
    obs_records = sorted(rover_records + ref_kept, key=lambda x: x[0])

    obs_lo = obs_records[0][0]
    imu_t0 = first_sensor_utc(sensors, ts1, IMU_TOPIC)
    cam_t0 = first_sensor_utc(sensors, ts1, CAM_TOPIC)
    # Anchor t0 to the earliest retained observation, not just the rover epoch:
    # the reference RINEX is sampled at 30 s and can legitimately bracket the
    # rover start. Burst ephemeris/antenna/DCB before that first retained obs so
    # every observation sees the same preloaded context as file-mode burst_load.
    sensor_starts = [v for v in (imu_t0, cam_t0) if v is not None and v >= obs_lo - args.sensor_margin]
    t0 = min([obs_lo] + sensor_starts)
    sensor_lo = t0 - args.sensor_margin
    sensor_hi = rover_hi + args.sensor_margin
    print(f"t0(utc)={t0:.3f}  rover=[{rover_lo:.3f},{rover_hi:.3f}]  "
          f"ref_kept={len(ref_kept)} (dropped {n_ref_dropped})  "
          f"sensor window=[{sensor_lo:.3f},{sensor_hi:.3f}]  leap={args.leap}s")

    burst_base_ns = int((t0 - 2.0) * 1e9)

    counts: dict[str, int] = {}
    spans: dict[str, list[float]] = {}

    def note(topic: str, ts_ns: int) -> None:
        counts[topic] = counts.get(topic, 0) + 1
        s = ts_ns * 1e-9
        if topic not in spans:
            spans[topic] = [s, s]
        else:
            spans[topic][0] = min(spans[topic][0], s)
            spans[topic][1] = max(spans[topic][1], s)

    n_drop = 0
    with Writer(out_dir, version=8) as writer:
        conns: dict[str, object] = {}

        def conn_for(topic: str, rtype: str):
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, rtype, typestore=ts2)
            return conns[topic]

        # Burst eph / ion / antenna / DCB up-front (+1 ms each, order preserved).
        for i, (topic, rtype, raw) in enumerate(burst_records):
            wt = burst_base_ns + i * 1_000_000
            writer.write(conn_for(topic, rtype), wt, raw)
            note(topic, wt)

        # Observations reclocked to internal UTC.
        for utc, topic, rtype, raw in obs_records:
            wt = int(utc * 1e9)
            writer.write(conn_for(topic, rtype), wt, raw)
            note(topic, wt)

        # Sensors: IMU + camera, header UTC (unshifted), bgr8 -> mono8.
        n_imu = n_cam = 0
        with Reader1(sensors) as r:
            print(f"Merging sensors {sensors.name} (IMU + left camera) ...")
            for c, t, raw in r.messages():
                if c.topic not in (IMU_TOPIC, CAM_TOPIC):
                    continue
                src = ts1.deserialize_ros1(raw, c.msgtype)
                utc = src.header.stamp.sec + src.header.stamp.nanosec * 1e-9
                if utc < sensor_lo or utc > sensor_hi:
                    n_drop += 1
                    continue
                wt = int(utc * 1e9)
                if c.topic == IMU_TOPIC:
                    msg2 = u.convert(ts2, "sensor_msgs/msg/Imu", src)
                    data = ts2.serialize_cdr(msg2, "sensor_msgs/msg/Imu")
                    writer.write(conn_for(IMU_TOPIC, "sensor_msgs/msg/Imu"), wt, data)
                    n_imu += 1
                else:
                    msg2 = u.bgr8_to_mono8(ts2, src, 0.0)
                    data = ts2.serialize_cdr(msg2, "sensor_msgs/msg/Image")
                    writer.write(conn_for(CAM_TOPIC, "sensor_msgs/msg/Image"), wt, data)
                    n_cam += 1
                note(c.topic, wt)

    manifest = {
        "converter": "scripts/ros2/finalize_rinex_bag.py",
        "converter_sha256": sha256_file(Path(__file__)),
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
        "topics": {
            topic: {
                "count": counts[topic],
                "record_time_start": spans[topic][0],
                "record_time_end": spans[topic][1],
            }
            for topic in sorted(counts)
        },
        "sensors_dropped_outside_window": n_drop,
        "reference_epochs_dropped_outside_drive": n_ref_dropped,
        "rover_drive_utc": [rover_lo, rover_hi],
    }
    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))

    print(f"Done -> {out_dir}")
    for topic in sorted(counts):
        print(f"  {topic:45s} {counts[topic]:8d}")
    print(f"  sensors dropped (outside window): {n_drop}")
    print(f"  manifest -> {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
