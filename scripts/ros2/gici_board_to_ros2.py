#!/usr/bin/env python3
"""Merge GICI board ROS 1 bags (from gici_tools) into one ROS 2 bag.

Input bags (under dataset dir):
  gnss_rover.bag, gnss_reference.bag, imu.bag, image.bag

Ephemeris is not converted to a bag (author tool segfaults on gnss_ephemeris.bin);
ros_gici_board_bag_hybrid_rrr.yaml loads eph from the *.bin at node start.

Topics for ros_gici_board_bag_hybrid_rrr.yaml:
  /gici/gnss_rover/observations
  /gici/gnss_reference/observations, /gici/gnss_reference/antenna_position
  /gici/imu_raw, /gici/image_raw

Reference antenna_position is bursted at bag start; obs/IMU/image times follow UTC.
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

REPO = Path(__file__).resolve().parents[2]
MSG_DIR = REPO / "ros2_wrapper" / "src" / "gici_ros2_msgs" / "msg"
ROS1_MSG_DIR = REPO / "ros_wrapper" / "src" / "gici" / "msg"

IMU_TOPIC = "/gici/imu_raw"
CAM_TOPIC = "/gici/image_raw"
BURST_SUFFIXES = ("/ephemerides", "/ionosphere_parameter", "/antenna_position")

GPS_EPOCH_UNIX = 315964800
SECONDS_PER_WEEK = 604800

FIELD_RENAME = {
    "snr": "SNR", "lli": "LLI", "l": "L", "p": "P", "d": "D",
    "m0": "M0", "omg0": "OMG0", "omgd": "OMGd", "a": "A",
}


def build_ros2_typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add = {}
    for f in sorted(MSG_DIR.glob("*.msg")):
        add.update(get_types_from_msg(f.read_text(), f"gici_ros2_msgs/msg/{f.stem}"))
    ts.register(add)
    return ts


def build_ros1_typestore():
    ts = get_typestore(Stores.ROS1_NOETIC)
    add = {}
    for f in sorted(ROS1_MSG_DIR.glob("*.msg")):
        add.update(get_types_from_msg(f.read_text(), f"gici_ros/msg/{f.stem}"))
    ts.register(add)
    return ts


def r1_to_r2_type(t: str) -> str:
    if t.startswith("gici_ros/msg/"):
        return "gici_ros2_msgs/msg/" + t.split("/")[-1]
    return t


def is_msg(v) -> bool:
    return hasattr(v, "__msgtype__")


def convert(ts2, r2type, src):
    cls = ts2.types[r2type]
    kwargs = {}
    for field in dataclasses.fields(cls):
        r2n = field.name
        r1n = FIELD_RENAME.get(r2n, r2n)
        val = getattr(src, r1n)
        if is_msg(val):
            kwargs[r2n] = convert(ts2, r1_to_r2_type(val.__msgtype__), val)
        elif isinstance(val, list) and val and is_msg(val[0]):
            sub = r1_to_r2_type(val[0].__msgtype__)
            kwargs[r2n] = [convert(ts2, sub, v) for v in val]
        else:
            kwargs[r2n] = val
    return cls(**kwargs)


def gpst_from_obs(msg) -> float:
    o = msg.observations[0]
    return GPS_EPOCH_UNIX + o.week * SECONDS_PER_WEEK + o.tow


def first_obs_time(bag: Path, ts1, leap: float) -> float | None:
    with Reader(bag) as r:
        for c, _t, raw in r.messages():
            if not c.topic.endswith("/observations"):
                continue
            src = ts1.deserialize_ros1(raw, c.msgtype)
            if not src.observations:
                continue
            return gpst_from_obs(src) - leap
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset-dir", required=True, help="GICI board dataset dir with *.bag")
    ap.add_argument("--out", required=True, help="Output ROS 2 bag directory")
    ap.add_argument("--force", action="store_true", help="Remove existing --out directory")
    ap.add_argument("--leap", type=float, default=18.0,
                    help="GPST-UTC leap seconds (default 18 for 2023 GICI board)")
    args = ap.parse_args()

    ds = Path(args.dataset_dir)
    out_dir = Path(args.out)
    if out_dir.exists():
        if args.force:
            import shutil
            shutil.rmtree(out_dir)
        else:
            print(f"ERROR: {out_dir} exists; use --force or remove first.", file=sys.stderr)
            return 1

    bags = {
        "gnss": [ds / n for n in ("gnss_rover.bag", "gnss_reference.bag")],
        "imu": ds / "imu.bag",
        "cam": ds / "image.bag",
    }
    for b in bags["gnss"] + [bags["imu"], bags["cam"]]:
        if not b.is_file():
            print(f"ERROR: missing {b}", file=sys.stderr)
            return 1

    ts2 = build_ros2_typestore()
    ts1 = build_ros1_typestore()

    t0 = first_obs_time(ds / "gnss_rover.bag", ts1, args.leap)
    if t0 is None:
        print("ERROR: no rover observations in gnss_rover.bag", file=sys.stderr)
        return 1
    burst_base_ns = int((t0 - 2.0) * 1e9)
    print(f"t0(utc)={t0:.3f}  leap={args.leap}s")

    counts: dict[str, int] = {}
    with Writer(out_dir, version=8) as writer:
        conns: dict[str, object] = {}

        def write_msg(topic: str, rtype: str, wt_ns: int, data: bytes) -> None:
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, rtype, typestore=ts2)
            writer.write(conns[topic], wt_ns, data)
            counts[topic] = counts.get(topic, 0) + 1

        burst_i = 0
        for bag in bags["gnss"]:
            with Reader(bag) as r:
                for c, _t, raw in r.messages():
                    src = ts1.deserialize_ros1(raw, c.msgtype)
                    r2type = r1_to_r2_type(c.msgtype)
                    msg2 = convert(ts2, r2type, src)
                    data = ts2.serialize_cdr(msg2, r2type)
                    if c.topic.endswith(BURST_SUFFIXES):
                        wt = burst_base_ns + burst_i * 1_000_000
                        burst_i += 1
                    else:
                        if not src.observations:
                            continue
                        wt = int((gpst_from_obs(src) - args.leap) * 1e9)
                    write_msg(c.topic, r2type, wt, data)

        for kind, bag in (("imu", bags["imu"]), ("cam", bags["cam"])):
            with Reader(bag) as r:
                for c, _t, raw in r.messages():
                    src = ts1.deserialize_ros1(raw, c.msgtype)
                    utc = src.header.stamp.sec + src.header.stamp.nanosec * 1e-9
                    wt = int(utc * 1e9)
                    if c.topic == IMU_TOPIC:
                        msg2 = convert(ts2, "sensor_msgs/msg/Imu", src)
                        data = ts2.serialize_cdr(msg2, "sensor_msgs/msg/Imu")
                        write_msg(IMU_TOPIC, "sensor_msgs/msg/Imu", wt, data)
                    elif c.topic == CAM_TOPIC:
                        msg2 = convert(ts2, "sensor_msgs/msg/Image", src)
                        data = ts2.serialize_cdr(msg2, "sensor_msgs/msg/Image")
                        write_msg(CAM_TOPIC, "sensor_msgs/msg/Image", wt, data)

    print(f"Done -> {out_dir}")
    for topic in sorted(counts):
        print(f"  {topic:50s} {counts[topic]:8d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
