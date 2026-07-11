#!/usr/bin/env python3
"""Convert UrbanNav TST/Whampoa ground-truth text to IE-style rows for ie_to_nmea."""

from __future__ import annotations

import argparse
from pathlib import Path


def dms_to_deg(d: float, m: float, s: float) -> float:
    sign = -1.0 if d < 0 else 1.0
    return sign * (abs(d) + m / 60.0 + s / 3600.0)


def convert(src: Path, dst: Path) -> int:
    rows: list[str] = []
    for line in src.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(("UTCTime", "(sec)", "GPSTime")):
            continue
        parts = line.split()
        if len(parts) < 19:
            continue
        try:
            utc = float(parts[0])
            week = int(float(parts[1]))
            gpst = float(parts[2])
            lat = dms_to_deg(float(parts[3]), float(parts[4]), float(parts[5]))
            lon = dms_to_deg(float(parts[6]), float(parts[7]), float(parts[8]))
            h = float(parts[9])
            roll = float(parts[16])
            pitch = float(parts[17])
            heading = float(parts[18])
        except ValueError:
            continue
        rows.append(
            f"{week} {gpst:.3f} {utc:.3f} {lon:.10f} {lat:.10f} {h:.3f} "
            f"{heading:.10f} {pitch:.10f} {roll:.10f}"
        )

    if not rows:
        raise RuntimeError(f"No ground-truth rows parsed from {src}")

    dst.write_text("\n".join(rows) + "\n")
    return len(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    args = ap.parse_args()
    n = convert(args.src, args.dst)
    print(f"Wrote {n} IE rows to {args.dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
