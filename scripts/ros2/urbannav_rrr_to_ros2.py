#!/usr/bin/env python3
"""Build ONE ROS 2 bag for UrbanNav RRR (GNSS + IMU + camera).

This merges:
  * the GICI GNSS ROS1 bags (rover / reference / ephemeris_{G,R,E,C}), whose
    messages use the ORIGINAL ``gici_ros/msg/*`` types (upper-case fields), and
  * the UrbanNav sensor ROS1 bag (``/imu/data`` = sensor_msgs/Imu,
    ``/zed2/camera/left/image_raw`` = sensor_msgs/Image).

Two things make RRR fusion actually work here:

1. TIME ALIGNMENT (the critical part).
   GICI reads the obs GPS week/tow, then converts to UTC via gpst2utc (gnss_common
   gpsTimeToUtcTime, = gpst - 18 leap seconds on 2021-05-17) and uses THAT as the
   GNSS measurement timestamp. It reads IMU/camera time straight from
   ``header.stamp``, which in the UrbanNav bags is already UTC (Unix). So the sensor
   stamps are ALREADY on the same timescale as GICI's GNSS timestamps and must NOT be
   shifted. (An earlier version added +18 s to the sensors, pushing them 18 s ahead
   of the GNSS-UTC timeline; the estimator then integrated IMU over an ~18 s gap
   during visual init and aborted, or threw every "late" GNSS epoch away.)

   We set each message's bag *record* timestamp equal to its GICI-internal UTC time
   (GNSS = gpst - ``--leap``, sensors = header as-is), so record==internal and
   ros2 bag play paces all three sensors on ONE shared clock. Otherwise the GNSS
   record clock and the sensor record clock drift ~1 s apart and the input-align
   buffer discards GNSS ("latency too large") -> no RTK, no solution.

2. EPHEMERIS BURST.
   The ephemeris bags are timestamped at each message's toe and span ~20 hours,
   while the drive is only ~13 min. We move all ephemeris / ionosphere /
   antenna-position messages to a short burst at the very start of the bag so the
   estimator has them before the first observation (like ``burst_load`` in
   file mode). GICI uses the message's internal fields, not the bag timestamp.

Field types are converted gici_ros -> gici_ros2_msgs (snake_case) and images are
converted bgr8 -> mono8 (what the wrapper feeds the tracker anyway), which also
cuts the bag size ~3x.

Usage:
  python3 scripts/ros2/urbannav_rrr_to_ros2.py \
      --gnss-dir /path/to/urbannav/medium \
      --sensors  /path/to/UrbanNav-..._sensors.bag \
      --out      output/ros2_urbannav/medium/rrr_ros2
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

REPO = Path(__file__).resolve().parents[2]
MSG_DIR = REPO / "ros2_wrapper" / "src" / "gici_ros2_msgs" / "msg"
ROS1_MSG_DIR = REPO / "ros_wrapper" / "src" / "gici" / "msg"

IMU_TOPIC = "/imu/data"
CAM_TOPIC = "/zed2/camera/left/image_raw"

# GPS epoch (1980-01-06) in Unix seconds; gpst = GPS_EPOCH_UNIX + week*604800 + tow,
# which is exactly what rtklib's gpst2time() yields and what GICI reads from each obs.
GPS_EPOCH_UNIX = 315964800
SECONDS_PER_WEEK = 604800

# ROS 2 (snake_case) -> ROS 1 (author upper-case) field-name overrides for gici msgs.
FIELD_RENAME = {
    "snr": "SNR", "lli": "LLI", "l": "L", "p": "P", "d": "D",
    "m0": "M0", "omg0": "OMG0", "omgd": "OMGd", "a": "A",
}

# GNSS topics that must be delivered up-front (static/slow setup data).
PREFIX_SUFFIXES = ("/ephemerides", "/ionosphere_parameter", "/antenna_position")


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
    """Recursively rebuild a ROS 2 message from a ROS 1 message object."""
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


def bgr8_to_mono8(ts2, src, stamp_offset: float):
    """Convert a ROS1 sensor_msgs/Image (bgr8) to a ROS2 mono8 Image."""
    h, w = int(src.height), int(src.width)
    buf = np.frombuffer(bytes(src.data), dtype=np.uint8).reshape(h, src.step)
    bgr = buf[:, : w * 3].reshape(h, w, 3).astype(np.float32)
    # OpenCV COLOR_BGR2GRAY weights (same as cv_bridge bgr8->mono8).
    gray = (0.114 * bgr[:, :, 0] + 0.587 * bgr[:, :, 1] + 0.299 * bgr[:, :, 2])
    gray = np.clip(gray + 0.5, 0, 255).astype(np.uint8).reshape(-1)

    Image = ts2.types["sensor_msgs/msg/Image"]
    Header = ts2.types["std_msgs/msg/Header"]
    Time = ts2.types["builtin_interfaces/msg/Time"]
    total = src.header.stamp.sec + src.header.stamp.nanosec * 1e-9 + stamp_offset
    sec = int(total)
    nsec = int(round((total - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    hdr = Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=src.header.frame_id)
    return Image(header=hdr, height=h, width=w, encoding="mono8",
                 is_bigendian=0, step=w, data=gray)


def gpst_from_obs(msg) -> float:
    """GPST (Unix seconds) of a GnssObservations message from its week/tow."""
    o = msg.observations[0]
    return GPS_EPOCH_UNIX + o.week * SECONDS_PER_WEEK + o.tow


def first_internal_time(bag: Path, ts1, kind: str, leap: float, topics=None):
    """Earliest GICI-internal UTC time: obs -> gpst(week,tow)-leap, sensor -> header."""
    with Reader(bag) as r:
        for c, t, raw in r.messages():
            if topics is not None and c.topic not in topics:
                continue
            src = ts1.deserialize_ros1(raw, c.msgtype)
            if kind == "obs":
                if not src.observations:
                    continue
                return gpst_from_obs(src) - leap
            else:  # sensor header (already UTC)
                return src.header.stamp.sec + src.header.stamp.nanosec * 1e-9
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gnss-dir", required=True,
                    help="Directory with gnss_rover.bag / gnss_reference.bag / gnss_ephemeris_*.bag")
    ap.add_argument("--sensors", required=True, help="UrbanNav ..._sensors.bag")
    ap.add_argument("--out", required=True, help="Output ROS 2 bag directory (must not exist)")
    ap.add_argument("--leap", type=float, default=18.0,
                    help="GPST-UTC leap seconds (default 18 for 2021). Used to convert the "
                         "GNSS gpst(week,tow) into the UTC timescale GICI actually uses "
                         "(gpst2utc), so GNSS record times line up with the UTC IMU/camera "
                         "header stamps. Sensor stamps are NOT shifted.")
    ap.add_argument("--window", type=float, default=800.0,
                    help="Keep only stream (obs/imu/camera) messages within this many seconds "
                         "of the drive start; drops the reference-obs tail so playback is ~drive "
                         "length (default 800 s ~= 13 min).")
    args = ap.parse_args()

    out_dir = Path(args.out)
    if out_dir.exists():
        print(f"ERROR: output {out_dir} already exists; remove it first.", file=sys.stderr)
        return 1

    gdir = Path(args.gnss_dir)
    gnss_bags = [gdir / n for n in (
        "gnss_rover.bag", "gnss_reference.bag",
        "gnss_ephemeris_G.bag", "gnss_ephemeris_R.bag",
        "gnss_ephemeris_E.bag", "gnss_ephemeris_C.bag")]
    sensors = Path(args.sensors)
    for b in gnss_bags + [sensors]:
        if not b.exists():
            print(f"ERROR: missing input {b}", file=sys.stderr)
            return 1

    ts2 = build_ros2_typestore()
    ts1 = build_ros1_typestore()

    # CRITICAL: set every message's bag RECORD time equal to its GICI-internal UTC
    # measurement time. GNSS reads time from week/tow and the core converts it with
    # gpst2utc; UrbanNav sensors are read from header.stamp, already UTC. Reclocking
    # record==internal lets ros2 bag play deliver the streams correctly interleaved.
    t0 = min(
        first_internal_time(gdir / "gnss_rover.bag", ts1, "obs", args.leap),
        first_internal_time(sensors, ts1, "sensor", args.leap, {IMU_TOPIC}),
        first_internal_time(sensors, ts1, "sensor", args.leap, {CAM_TOPIC}),
    )
    cutoff = t0 + args.window
    burst_base_ns = int((t0 - 2.0) * 1e9)  # ephemeris burst just before first obs
    print(f"drive start t0(utc)={t0:.3f}  cutoff={cutoff:.3f}  leap={args.leap}s")

    n_eph = n_obs = n_imu = n_cam = n_drop = 0
    with Writer(out_dir, version=8) as writer:  # rosbag2 storage version 8 (Humble)
        conns: dict[str, object] = {}

        def conn_for(topic, rtype):
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, rtype, typestore=ts2)
            return conns[topic]

        # --- GNSS: ephemeris/ion/antenna bursted first, observations reclocked to gpst ---
        burst_i = 0
        for bag in gnss_bags:
            with Reader(bag) as r:
                print(f"Converting {bag.name} ...")
                for c, t, raw in r.messages():
                    src = ts1.deserialize_ros1(raw, c.msgtype)
                    r2type = r1_to_r2_type(c.msgtype)
                    msg2 = convert(ts2, r2type, src)
                    data = ts2.serialize_cdr(msg2, r2type)
                    if c.topic.endswith(PREFIX_SUFFIXES):
                        wt = burst_base_ns + burst_i * 1_000_000  # +1 ms each
                        burst_i += 1
                        n_eph += 1
                    else:  # observations -> record time = GICI-internal UTC = gpst-leap
                        if not src.observations:
                            continue
                        utc = gpst_from_obs(src) - args.leap
                        if utc > cutoff:
                            n_drop += 1
                            continue
                        wt = int(utc * 1e9)
                        n_obs += 1
                    writer.write(conn_for(c.topic, r2type), wt, data)

        # --- Sensors: IMU + camera. Header stamps are already UTC (same scale as
        #     GICI's GNSS timestamps) -> NOT shifted; record time = header time. ---
        with Reader(sensors) as r:
            print(f"Converting {sensors.name} (IMU + left camera) ...")
            for c, t, raw in r.messages():
                if c.topic not in (IMU_TOPIC, CAM_TOPIC):
                    continue
                src = ts1.deserialize_ros1(raw, c.msgtype)
                utc = src.header.stamp.sec + src.header.stamp.nanosec * 1e-9
                if utc > cutoff:
                    n_drop += 1
                    continue
                wt = int(utc * 1e9)
                if c.topic == IMU_TOPIC:
                    msg2 = convert(ts2, "sensor_msgs/msg/Imu", src)
                    data = ts2.serialize_cdr(msg2, "sensor_msgs/msg/Imu")
                    writer.write(conn_for(IMU_TOPIC, "sensor_msgs/msg/Imu"), wt, data)
                    n_imu += 1
                else:
                    msg2 = bgr8_to_mono8(ts2, src, 0.0)  # keep header UTC (no shift)
                    data = ts2.serialize_cdr(msg2, "sensor_msgs/msg/Image")
                    writer.write(conn_for(CAM_TOPIC, "sensor_msgs/msg/Image"), wt, data)
                    n_cam += 1
                if (n_imu + n_cam) % 50000 == 0:
                    print(f"  ... imu={n_imu} cam={n_cam}")

    print(f"Done -> {out_dir}")
    print(f"  ephemeris/ion/antenna (bursted): {n_eph}")
    print(f"  observations:                    {n_obs}")
    print(f"  imu:                             {n_imu}")
    print(f"  camera (mono8):                  {n_cam}")
    print(f"  dropped (outside window):        {n_drop}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
