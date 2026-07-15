#!/usr/bin/env python3
"""Convert a ROS 2 canonical UrbanNav RRR bag to one ROS 1 Noetic bag.

The ROS 2 canonical bag (from scripts/ros2/finalize_rinex_bag.py) already has
GNSS record times aligned to GICI-internal UTC and the same HKKT+DCB inputs as
file-mode Table V. This script re-serializes messages as gici_ros (ROS 1) types
so gici_ros_main can replay them with rosbag play.

Usage:
  python3 scripts/ros1/canonical_ros2_to_ros1_bag.py \\
      --in  output/ros2_urbannav_rrr_canonical/medium/canonical_bag \\
      --out output/ros1_urbannav_rrr_canonical/medium/canonical.bag
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts" / "ros2"))
import urbannav_rrr_to_ros2 as u  # noqa: E402

from rosbags.rosbag1 import Writer as Writer1  # noqa: E402
from rosbags.rosbag2 import Reader as Reader2  # noqa: E402

R2_TO_R1 = {v: k for k, v in u.FIELD_RENAME.items()}


def r2_to_r1_type(t: str) -> str:
    if t.startswith("gici_ros2_msgs/msg/"):
        return "gici_ros/msg/" + t.split("/")[-1]
    return t


def is_msg(v) -> bool:
    return hasattr(v, "__msgtype__")


def convert_r2_to_r1(ts1, r1type, src):
    if r1type == "std_msgs/Header" or r1type == "std_msgs/msg/Header":
        return stamp_r1(ts1, src)
    cls = ts1.types[r1type]
    kwargs = {}
    for field in dataclasses.fields(cls):
        r1n = field.name
        r2n = R2_TO_R1.get(r1n, r1n)
        if not hasattr(src, r2n):
            if r1n == "seq":
                kwargs[r1n] = 0
                continue
            continue
        val = getattr(src, r2n)
        if is_msg(val):
            kwargs[r1n] = convert_r2_to_r1(ts1, r2_to_r1_type(val.__msgtype__), val)
        elif isinstance(val, list) and val and is_msg(val[0]):
            sub = r2_to_r1_type(val[0].__msgtype__)
            kwargs[r1n] = [convert_r2_to_r1(ts1, sub, v) for v in val]
        else:
            kwargs[r1n] = val
    return cls(**kwargs)


def stamp_r1(ts1, hdr2):
    hdr_type = "std_msgs/msg/Header" if "std_msgs/msg/Header" in ts1.types else "std_msgs/Header"
    Header = ts1.types[hdr_type]
    Time = ts1.types.get("builtin_interfaces/Time") or ts1.types.get("rosgraph_msgs/Time")
    if Time is not None:
        stamp = Time(sec=hdr2.stamp.sec, nanosec=hdr2.stamp.nanosec)
    else:
        stamp = type("Stamp", (), {"sec": hdr2.stamp.sec, "nanosec": hdr2.stamp.nanosec})()
    fields = {f.name for f in dataclasses.fields(Header)}
    kwargs = {"stamp": stamp, "frame_id": hdr2.frame_id}
    if "seq" in fields:
        kwargs["seq"] = 0
    return Header(**kwargs)


def ros1_type(ts1, name: str) -> str:
    if f"sensor_msgs/msg/{name}" in ts1.types:
        return f"sensor_msgs/msg/{name}"
    return f"sensor_msgs/{name}"


def convert_imu(ts1, src):
    r1type = ros1_type(ts1, "Imu")
    Imu = ts1.types[r1type]
    return Imu(
        header=stamp_r1(ts1, src.header),
        orientation=src.orientation,
        orientation_covariance=src.orientation_covariance,
        angular_velocity=src.angular_velocity,
        angular_velocity_covariance=src.angular_velocity_covariance,
        linear_acceleration=src.linear_acceleration,
        linear_acceleration_covariance=src.linear_acceleration_covariance,
    )


def convert_image(ts1, src):
    r1type = ros1_type(ts1, "Image")
    Image = ts1.types[r1type]
    raw = src.data
    if isinstance(raw, (bytes, bytearray)):
        buf = np.frombuffer(raw, dtype=np.uint8)
    else:
        buf = np.asarray(raw, dtype=np.uint8)
    return Image(
        header=stamp_r1(ts1, src.header),
        height=src.height,
        width=src.width,
        encoding=src.encoding,
        is_bigendian=src.is_bigendian,
        step=src.step,
        data=buf,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="in_dir", required=True, help="ROS 2 canonical bag directory")
    ap.add_argument("--out", required=True, help="Output ROS 1 .bag file (must not exist)")
    args = ap.parse_args()

    in_dir = Path(args.in_dir)
    out_bag = Path(args.out)
    if not in_dir.is_dir():
        print(f"ERROR: missing input bag dir {in_dir}", file=sys.stderr)
        return 1
    if out_bag.exists():
        print(f"ERROR: {out_bag} exists; remove first.", file=sys.stderr)
        return 1

    ts2 = u.build_ros2_typestore()
    ts1 = u.build_ros1_typestore()
    out_bag.parent.mkdir(parents=True, exist_ok=True)

    counts: dict[str, int] = {}
    with Writer1(out_bag) as writer:
        conns: dict[str, object] = {}

        def conn_for(topic: str, rtype: str):
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, rtype, typestore=ts1)
            return conns[topic]

        with Reader2(in_dir) as reader:
            for c, t, raw in reader.messages():
                topic = c.topic
                if c.msgtype.startswith("gici_ros2_msgs/"):
                    r1type = r2_to_r1_type(c.msgtype)
                    msg2 = ts2.deserialize_cdr(raw, c.msgtype)
                    msg1 = convert_r2_to_r1(ts1, r1type, msg2)
                    data = ts1.serialize_ros1(msg1, r1type)
                elif c.msgtype == "sensor_msgs/msg/Imu":
                    msg2 = ts2.deserialize_cdr(raw, c.msgtype)
                    r1type = ros1_type(ts1, "Imu")
                    msg1 = convert_imu(ts1, msg2)
                    data = ts1.serialize_ros1(msg1, r1type)
                elif c.msgtype == "sensor_msgs/msg/Image":
                    msg2 = ts2.deserialize_cdr(raw, c.msgtype)
                    r1type = ros1_type(ts1, "Image")
                    msg1 = convert_image(ts1, msg2)
                    data = ts1.serialize_ros1(msg1, r1type)
                else:
                    print(f"WARN: skip unknown type {c.msgtype} on {topic}", file=sys.stderr)
                    continue
                writer.write(conn_for(topic, r1type), t, data)
                counts[topic] = counts.get(topic, 0) + 1

    print(f"Done -> {out_bag}")
    for topic in sorted(counts):
        print(f"  {topic:45s} {counts[topic]:8d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
