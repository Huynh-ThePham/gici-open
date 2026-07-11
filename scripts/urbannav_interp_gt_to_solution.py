#!/usr/bin/env python3
"""Interpolate 1 Hz UrbanNav IE ground truth to solution GPST timestamps."""

from __future__ import annotations

import argparse
from pathlib import Path


def load_ie_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 9:
            continue
        rows.append(
            {
                "week": int(parts[0]),
                "gpst": float(parts[1]),
                "utc": float(parts[2]),
                "lon": float(parts[3]),
                "lat": float(parts[4]),
                "h": float(parts[5]),
                "heading": float(parts[6]),
                "pitch": float(parts[7]),
                "roll": float(parts[8]),
            }
        )
    if not rows:
        raise RuntimeError(f"No IE rows in {path}")
    return rows


def interp_angle(a0: float, a1: float, alpha: float) -> float:
    import math

    da = (a1 - a0 + 180.0) % 360.0 - 180.0
    return (a0 + alpha * da) % 360.0


def interp_row(rows: list[dict[str, float]], t: float) -> dict[str, float] | None:
    if t < rows[0]["gpst"] or t > rows[-1]["gpst"]:
        return None
    lo = 0
    hi = len(rows) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if rows[mid]["gpst"] <= t:
            lo = mid
        else:
            hi = mid
    if rows[hi]["gpst"] < t:
        return None
    if rows[lo]["gpst"] == t:
        return rows[lo]
    if rows[hi]["gpst"] == t:
        return rows[hi]
    t0 = rows[lo]["gpst"]
    t1 = rows[hi]["gpst"]
    if t1 <= t0:
        return rows[lo]
    alpha = (t - t0) / (t1 - t0)
    out = dict(rows[lo])
    out["gpst"] = t
    out["utc"] = rows[lo]["utc"] + alpha * (rows[hi]["utc"] - rows[lo]["utc"])
    for key in ("lon", "lat", "h", "pitch", "roll"):
        out[key] = rows[lo][key] + alpha * (rows[hi][key] - rows[lo][key])
    out["heading"] = interp_angle(rows[lo]["heading"], rows[hi]["heading"], alpha)
    return out


def write_ie_row(row: dict[str, float]) -> str:
    return (
        f"{row['week']} {row['gpst']:.3f} {row['utc']:.3f} "
        f"{row['lon']:.10f} {row['lat']:.10f} {row['h']:.3f} "
        f"{row['heading']:.10f} {row['pitch']:.10f} {row['roll']:.10f}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ie_gt", type=Path)
    ap.add_argument("solution", type=Path)
    ap.add_argument("out_ie", type=Path)
    ap.add_argument("--gps-week-day-offset", type=float, default=86400.0)
    args = ap.parse_args()

    rows = load_ie_rows(args.ie_gt)
    times: list[float] = []
    for line in args.solution.read_text(errors="replace").splitlines():
        if not line.startswith("$GPGGA,"):
            continue
        token = line.split(",")[1]
        if not token:
            continue
        hh = int(token[0:2])
        mm = int(token[2:4])
        ss = float(token[4:])
        sod = hh * 3600 + mm * 60 + ss
        times.append(args.gps_week_day_offset + sod + 18.0)

    if not times:
        raise RuntimeError(f"No GPGGA epochs in {args.solution}")

    out_lines: list[str] = []
    for t in times:
        row = interp_row(rows, t)
        if row is None:
            continue
        out_lines.append(write_ie_row(row))

    if not out_lines:
        raise RuntimeError("No interpolated GT rows for solution timestamps")

    args.out_ie.write_text("\n".join(out_lines) + "\n")
    print(f"Wrote {len(out_lines)} interpolated IE rows to {args.out_ie}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
