#!/usr/bin/env python3
"""Downsample GICI NMEA solution to UrbanNav GT integer-second timestamps."""

from __future__ import annotations

import argparse
from pathlib import Path


def parse_gpst(token: str, gps_week_day_offset: float, leap: float = 18.0) -> float:
    hh = int(token[0:2])
    mm = int(token[2:4])
    ss = float(token[4:])
    return gps_week_day_offset + hh * 3600 + mm * 60 + ss + leap


def load_gt_times(gt_raw: Path) -> list[float]:
    times: list[float] = []
    for line in gt_raw.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(("UTCTime", "(sec)", "GPSTime")):
            continue
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            times.append(float(parts[2]))
        except ValueError:
            continue
    if not times:
        raise RuntimeError(f"No GT timestamps in {gt_raw}")
    return times


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("solution", type=Path)
    ap.add_argument("gt_raw", type=Path)
    ap.add_argument("out_solution", type=Path)
    ap.add_argument("--gps-week-day-offset", type=float, default=86400.0)
    args = ap.parse_args()

    gt_times = load_gt_times(args.gt_raw)
    blocks: list[list[str]] = []
    current: list[str] = []
    for line in args.solution.read_text(errors="replace").splitlines():
        if line.startswith("$GPGGA,"):
            if current:
                blocks.append(current)
            current = [line]
        elif current:
            current.append(line)
    if current:
        blocks.append(current)

    if not blocks:
        raise RuntimeError(f"No GPGGA epochs in {args.solution}")

    by_gpst: dict[float, list[str]] = {}
    for block in blocks:
        gga = block[0]
        token = gga.split(",")[1]
        gpst = parse_gpst(token, args.gps_week_day_offset)
        by_gpst[gpst] = block

    selected: list[list[str]] = []
    for t in gt_times:
        if t in by_gpst:
            selected.append(by_gpst[t])
            continue
        nearest = min(by_gpst.keys(), key=lambda k: abs(k - t))
        if abs(nearest - t) <= 0.05:
            selected.append(by_gpst[nearest])

    if not selected:
        raise RuntimeError("No solution epochs matched GT timestamps")

    out_lines: list[str] = []
    for block in selected:
        out_lines.extend(block)
    args.out_solution.write_text("\n".join(out_lines) + "\n")
    print(f"Wrote {len(selected)} epochs to {args.out_solution}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
