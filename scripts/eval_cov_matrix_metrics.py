#!/usr/bin/env python3
"""Summarize [vaar-fast] matrix diagnostics for the paper T1 table.

Requires a NEW board log from the binary that emits max_diag_rel / gen_eig_* .
Rejects old logs that only have rel_err/tr_*.
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
from pathlib import Path


RX = re.compile(
    r"\[vaar-fast\].*?fast_ok=(\d).*?rel_err=([0-9.eE+-]+).*?"
    r"tr_fast=([0-9.eE+-]+) tr_ceres=([0-9.eE+-]+).*?"
    r"max_diag_rel=([0-9.eE+-nanNAN]+).*?min_eig_diff=([0-9.eE+-nanNAN]+).*?"
    r"psd_diff=(-?\d).*?gen_eig_min=([0-9.eE+-nanNAN]+).*?gen_eig_max=([0-9.eE+-nanNAN]+).*?"
    r"fast_ms=([0-9.eE+-]+) ceres_ms=([0-9.eE+-]+)"
)


def _f(s: str) -> float | None:
    try:
        v = float(s)
        return None if math.isnan(v) else v
    except ValueError:
        return None


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    ys = sorted(xs)
    k = (len(ys) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return ys[lo]
    return ys[lo] * (hi - k) + ys[hi] * (k - lo)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path, help="run.stderr with [vaar-fast] lines")
    args = ap.parse_args()
    text = args.log.read_text(errors="ignore")
    if "max_diag_rel=" not in text:
        raise SystemExit(
            f"REJECT: {args.log} has no max_diag_rel — this is an OLD log. "
            "Re-run scripts/run_va_1_1.sh with the rebuilt build/gici_main."
        )

    n = n_ok = n_frob = n_psd = n_psd_ok = 0
    frobs, diags, mineigs, genmaxs, fast_ms, ceres_ms = [], [], [], [], [], []
    for line in text.splitlines():
        m = RX.search(line)
        if not m:
            continue
        n += 1
        ok, rel, trf, trc, mdr, med, psd, gmin, gmax, fms, cms = m.groups()
        fast_ms.append(float(fms))
        ceres_ms.append(float(cms))
        if ok != "1":
            continue
        n_ok += 1
        r = float(rel)
        frobs.append(r)
        if r <= 1e-3:
            n_frob += 1
        if (v := _f(mdr)) is not None:
            diags.append(v)
        if (v := _f(med)) is not None:
            mineigs.append(v)
        if (v := _f(gmax)) is not None:
            genmaxs.append(v)
        if psd != "-1":
            n_psd += 1
            if psd == "1":
                n_psd_ok += 1

    if n == 0:
        raise SystemExit("no parseable [vaar-fast] lines with matrix fields")

    print(f"log: {args.log}")
    print(f"epochs={n}  fast_ok={n_ok} ({100*n_ok/n:.2f}%)")
    print(f"Frobenius <=1e-3: {100*n_frob/max(n_ok,1):.1f}% of usable")
    print(f"Frobenius p50/p95/max: {pct(frobs,50):.3e} / {pct(frobs,95):.3e} / {max(frobs):.3e}")
    print(f"max_diag_rel p50/p95/max: {pct(diags,50):.3e} / {pct(diags,95):.3e} / {max(diags):.3e}")
    print(f"min_eig(Qf-Qc) min/p50: {min(mineigs):.3e} / {pct(mineigs,50):.3e}")
    print(f"psd_diff rate (Loewner Qf>=Qc): {100*n_psd_ok/max(n_psd,1):.1f}% of scored")
    print(f"gen_eig_max (worst dir. var ratio) p50/p95/max: "
          f"{pct(genmaxs,50):.4f} / {pct(genmaxs,95):.4f} / {max(genmaxs):.4f}")
    print(f"fast_ms mean/med/p95/p99/max: "
          f"{statistics.mean(fast_ms):.1f} / {pct(fast_ms,50):.1f} / "
          f"{pct(fast_ms,95):.1f} / {pct(fast_ms,99):.1f} / {max(fast_ms):.1f}")
    print(f"ceres_ms mean/med/p95/max: "
          f"{statistics.mean(ceres_ms):.1f} / {pct(ceres_ms,50):.1f} / "
          f"{pct(ceres_ms,95):.1f} / {max(ceres_ms):.1f}")
    print(f">50ms fast: {100*sum(x>50 for x in fast_ms)/n:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
