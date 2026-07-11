#!/usr/bin/env python3
"""Convert UrbanNav GICI GNSS ROS1 bags into a single ROS 2 bag.

The UrbanNav dataset ships ROS1 bags whose messages use the ORIGINAL author type
``gici_ros/msg/*`` (with upper-case fields SNR/LLI/L/P/D/A/M0/OMG0/OMGd). ROS 2's
rosidl forbids upper-case field names, so this repo's ``gici_ros2_msgs`` renames them
to snake_case. This tool deserializes the ROS1 messages and re-serializes them as
``gici_ros2_msgs/msg/*`` (CDR) into one merged ROS 2 bag, keeping topic names intact
(e.g. ``/gici/gnss_rover/observations``) so the ported gici_ros2 node subscribes as-is.

Usage:
  python3 scripts/ros2/urbannav_gnss_bag_to_ros2.py \
      --in gnss_rover.bag gnss_reference.bag gnss_ephemeris_G.bag ... \
      --out /path/to/output_ros2_bag_dir

The output is a directory (ROS 2 sqlite3 bag). Play it with:
  ros2 bag play <output_ros2_bag_dir>
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
# Original author ROS1 .msg files (upper-case fields), used to deserialize the bags
ROS1_MSG_DIR = REPO / "ros_wrapper" / "src" / "gici" / "msg"

# ROS 2 (snake_case) -> ROS 1 (author upper-case) field-name overrides.
# All other fields share the same name across both message sets.
FIELD_RENAME = {
    "snr": "SNR",
    "lli": "LLI",
    "l": "L",
    "p": "P",
    "d": "D",
    "m0": "M0",
    "omg0": "OMG0",
    "omgd": "OMGd",
    "a": "A",
}


def build_ros2_typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    add_types = {}
    for msg_file in sorted(MSG_DIR.glob("*.msg")):
        name = msg_file.stem
        text = msg_file.read_text()
        add_types.update(get_types_from_msg(text, f"gici_ros2_msgs/msg/{name}"))
    ts.register(add_types)
    return ts


def build_ros1_typestore():
    ts = get_typestore(Stores.ROS1_NOETIC)
    add_types = {}
    for msg_file in sorted(ROS1_MSG_DIR.glob("*.msg")):
        name = msg_file.stem
        text = msg_file.read_text()
        add_types.update(get_types_from_msg(text, f"gici_ros/msg/{name}"))
    ts.register(add_types)
    return ts


def ros1_to_ros2_typename(ros1_type: str) -> str:
    # gici_ros/msg/GnssObservations -> gici_ros2_msgs/msg/GnssObservations
    if ros1_type.startswith("gici_ros/msg/"):
        return "gici_ros2_msgs/msg/" + ros1_type.split("/")[-1]
    return ros1_type  # std_msgs/msg/Header, builtin_interfaces/msg/Time, ...


def is_msg(value) -> bool:
    return hasattr(value, "__msgtype__")


def convert(ts2, ros2_type: str, src):
    """Recursively build a ROS 2 message object from a ROS 1 message object."""
    cls = ts2.types[ros2_type]
    kwargs = {}
    for field in dataclasses.fields(cls):
        r2name = field.name
        r1name = FIELD_RENAME.get(r2name, r2name)
        value = getattr(src, r1name)
        if is_msg(value):
            sub_r2 = ros1_to_ros2_typename(value.__msgtype__)
            kwargs[r2name] = convert(ts2, sub_r2, value)
        elif isinstance(value, list) and value and is_msg(value[0]):
            sub_r2 = ros1_to_ros2_typename(value[0].__msgtype__)
            kwargs[r2name] = [convert(ts2, sub_r2, v) for v in value]
        else:
            # primitive scalar, string, numpy array, or empty list -> pass through
            kwargs[r2name] = value
    return cls(**kwargs)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="inputs", nargs="+", required=True,
                    help="Input ROS1 .bag files (rover/reference/ephemeris_*)")
    ap.add_argument("--out", dest="output", required=True,
                    help="Output ROS 2 bag directory (must not exist)")
    ap.add_argument("--max-gap", type=float, default=1.0,
                    help="Cap inter-message idle gaps to this many seconds when reclocking "
                         "(default 1.0, which preserves the native ~1 Hz GNSS cadence and only "
                         "compresses the day-long gaps in the ephemeris bags). Without reclocking, "
                         "real-time playback would take ~21h. Reclocking preserves message ORDER "
                         "(GICI uses the GNSS week/tow inside each message, not the bag timestamp). "
                         "Do NOT set this very small: playing GNSS much faster than ~1 Hz can race "
                         "the GICI core's multi-threaded pipeline.")
    ap.add_argument("--no-reclock", action="store_true",
                    help="Keep original bag timestamps (playback spans the full recording).")
    args = ap.parse_args()

    out_dir = Path(args.output)
    if out_dir.exists():
        print(f"ERROR: output {out_dir} already exists; remove it first.", file=sys.stderr)
        return 1

    ts2 = build_ros2_typestore()
    ts1 = build_ros1_typestore()

    # Deserialize + re-serialize everything first, collecting (timestamp, topic, type, cdr)
    records: list[tuple[int, str, str, bytes]] = []
    for bag_path in args.inputs:
        bag_path = Path(bag_path)
        if not bag_path.exists():
            print(f"WARN: missing {bag_path}, skipping", file=sys.stderr)
            continue
        with Reader(bag_path) as reader:
            print(f"Converting {bag_path.name} ...")
            for conn, timestamp, raw in reader.messages():
                src = ts1.deserialize_ros1(raw, conn.msgtype)
                r2type = ros1_to_ros2_typename(conn.msgtype)
                msg2 = convert(ts2, r2type, src)
                records.append((timestamp, conn.topic, r2type,
                                ts2.serialize_cdr(msg2, r2type)))

    # Sort by original timestamp so playback order is correct across all topics
    records.sort(key=lambda r: r[0])

    # Optionally reclock: cap idle gaps so playback is fast but order is preserved
    if not args.no_reclock and records:
        max_gap_ns = int(args.max_gap * 1e9)
        reclocked = []
        prev_orig = records[0][0]
        cur = records[0][0]
        for i, (ts, topic, rtype, data) in enumerate(records):
            if i > 0:
                gap = ts - prev_orig
                if gap < 0:
                    gap = 0
                cur += min(gap, max_gap_ns)
                prev_orig = ts
            reclocked.append((cur, topic, rtype, data))
        records = reclocked

    total = 0
    # Humble's rosbag2 uses metadata/storage format version 8
    with Writer(out_dir, version=8) as writer:
        conn_cache: dict[str, object] = {}
        for ts, topic, rtype, data in records:
            if topic not in conn_cache:
                conn_cache[topic] = writer.add_connection(topic, rtype, typestore=ts2)
            writer.write(conn_cache[topic], ts, data)
            total += 1

    print(f"Done. Wrote {total} messages to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
