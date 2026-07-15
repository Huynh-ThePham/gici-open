#!/usr/bin/env python3
"""Downsample UrbanNav NMEA solution to ~1 Hz for paper Table V evaluation."""

from __future__ import annotations

import argparse
from pathlib import Path


def downsample_1hz(src: Path, dst: Path) -> int:
    """Keep the first GPGGA block per whole UTC second."""
    lines = src.read_text(errors="ignore").splitlines()
    blocks: list[list[str]] = []
    current: list[str] = []

    def flush() -> None:
        nonlocal current
        if any("GPGGA" in ln for ln in current):
            blocks.append(current)
        current = []

    for line in lines:
        if line.startswith("$GPRMC") and current:
            flush()
        current.append(line)
    flush()

    def second_key(block: list[str]) -> str | None:
        for ln in block:
            if "GPGGA" in ln:
                parts = ln.split(",")
                if len(parts) > 1 and parts[1]:
                    # hhmmss.sss -> integer second bucket
                    t = float(parts[1])
                    return f"{int(t // 10000):02d}{int((t % 10000) // 100):02d}{int(t % 100):02d}"
        return None

    picked: list[list[str]] = []
    seen: set[str] = set()
    for block in blocks:
        key = second_key(block)
        if key is None or key in seen:
            continue
        seen.add(key)
        picked.append(block)

    out_lines: list[str] = []
    for block in picked:
        out_lines.extend(block)

    dst.write_text("\n".join(out_lines) + ("\n" if out_lines else ""))
    return len(picked)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    args = ap.parse_args()
    n = downsample_1hz(args.src, args.dst)
    print(f"Downsampled {args.src} -> {args.dst}: {n} epochs @ ~1 Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
