#!/usr/bin/env python3
"""Convert UrbanNav NMEA (GGA/RMC/ESA) to TUM with a scene-local ENU origin.

Author nmea_to_tum hardcodes a Shanghai reference (GICI dataset 1.1). UrbanNav
scenes are in Hong Kong; using the first epoch as the ENU origin matches the
intent of evo_ape --align while avoiding latitude-dependent ENU distortion.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def dmm_to_deg(dmm: float) -> float:
    sign = -1.0 if dmm < 0 else 1.0
    dmm = abs(dmm)
    return sign * (math.floor(dmm / 100.0) + (dmm % 100.0) / 60.0)


def llh_to_ecef(lat_deg: float, lon_deg: float, h_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * math.cos(lon)
    y = (n + h_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + h_m) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float, y: float, z: float,
    ref_ecef: tuple[float, float, float],
    ref_lat_deg: float, ref_lon_deg: float,
) -> tuple[float, float, float]:
    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)
    dx, dy, dz = x - ref_ecef[0], y - ref_ecef[1], z - ref_ecef[2]
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def parse_nmea_epochs(path: Path) -> list[dict]:
    sol: dict | None = None
    pending_esa: dict | None = None
    epochs: list[dict] = []

    for raw in path.read_text(errors="ignore").splitlines():
        line = raw.strip()
        if not line.startswith("$"):
            continue
        body = line[1:]
        star = body.find("*")
        if star >= 0:
            body = body[:star]
        parts = body.split(",")
        tag = parts[0][2:5] if len(parts[0]) >= 5 else ""

        if tag == "RMC":
            # Date lives on RMC; GGA time refined later.
            if len(parts) < 10:
                continue
            sol = {"date": parts[9], "rmc_time": parts[1]}
            continue

        if tag == "GGA":
            if sol is None or len(parts) < 11:
                continue
            lat = dmm_to_deg(float(parts[2]))
            if parts[3] == "S":
                lat = -lat
            lon = dmm_to_deg(float(parts[4]))
            if parts[5] == "W":
                lon = -lon
            msl = float(parts[9]) if parts[9] else 0.0
            geoid = float(parts[11]) if len(parts) > 11 and parts[11] else 0.0
            h_ell = msl + geoid
            epochs.append(
                {
                    "utc_time": parts[1],
                    "date": sol["date"],
                    "lat": lat,
                    "lon": lon,
                    "h": h_ell,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                }
            )
            pending_esa = epochs[-1]
            continue

        if tag == "ESA" and pending_esa is not None and parts[1] == pending_esa["utc_time"]:
            if len(parts) >= 7:
                pending_esa["roll"] = math.radians(float(parts[4]))
                pending_esa["pitch"] = math.radians(float(parts[5]))
                pending_esa["yaw"] = math.radians(float(parts[6]))

    return epochs


def utc_to_unix(date_ddmmyy: str, utc_hhmmss: str) -> float:
    day = int(date_ddmmyy[:2])
    month = int(date_ddmmyy[2:4])
    year = int(date_ddmmyy[4:6])
    year += 2000 if year < 80 else 1900
    hh = int(float(utc_hhmmss) // 10000)
    mm = int((float(utc_hhmmss) % 10000) // 100)
    ss = float(utc_hhmmss) - hh * 10000 - mm * 100
    import datetime as dt

    return dt.datetime(year, month, day, hh, mm, int(ss), int((ss % 1) * 1e6)).timestamp() + (ss % 1)


def convert(path: Path, out_path: Path, ref_epoch: dict | None = None) -> int:
    epochs = parse_nmea_epochs(path)
    if not epochs:
        raise RuntimeError(f"No GGA epochs in {path}")

    ref = ref_epoch if ref_epoch is not None else epochs[0]
    ref_ecef = llh_to_ecef(ref["lat"], ref["lon"], ref["h"])

    lines = ["# timestamp tx ty tz qx qy qz qw"]
    for ep in epochs:
        ts = utc_to_unix(ep["date"], ep["utc_time"])
        ecef = llh_to_ecef(ep["lat"], ep["lon"], ep["h"])
        e, n, u = ecef_to_enu(*ecef, ref_ecef, ref["lat"], ref["lon"])
        qx, qy, qz, qw = euler_to_quat(ep["roll"], ep["pitch"], ep["yaw"])
        lines.append(f"{ts:.4f} {e:.6f} {n:.6f} {u:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}")

    out_path.write_text("\n".join(lines) + "\n")
    return len(epochs)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("nmea", type=Path)
    ap.add_argument("out_tum", type=Path, nargs="?", default=None)
    ap.add_argument(
        "--ref-nmea",
        type=Path,
        default=None,
        help="Use first GGA epoch of this NMEA as the ENU origin (e.g. aligned GT)",
    )
    args = ap.parse_args()
    out = args.out_tum or args.nmea.with_suffix(args.nmea.suffix + ".tum")
    ref_epoch = None
    if args.ref_nmea is not None:
        ref_epochs = parse_nmea_epochs(args.ref_nmea)
        if not ref_epochs:
            raise SystemExit(f"No GGA epochs in ref NMEA {args.ref_nmea}")
        ref_epoch = ref_epochs[0]
    n = convert(args.nmea, out, ref_epoch=ref_epoch)
    print(f"Wrote {n} poses to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
