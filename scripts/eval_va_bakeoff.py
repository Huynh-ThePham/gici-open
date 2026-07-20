#!/usr/bin/env python3
"""Rank the VA bake-off: baseline + VA variants, one run each, on one dataset.

Reads <OUT>/<tag>/<ds>/metrics.json for tag in bl,va,va3,va4,rfva,rfcva and prints
a ranked table (by horizontal RMSE) plus the vertical / yaw / fix-rate columns so
the "best" pick is transparent. Writes <OUT>/bakeoff_ranking.json.
"""
from __future__ import annotations
import json, sys
from pathlib import Path

ORDER = ["bl", "va", "va3", "va4", "rfva", "rfcva"]
LABEL = {"bl": "baseline", "va": "VA-v2 gate", "va3": "VA-v3 soft",
         "va4": "VA-v4 dose", "rfva": "RFVA Tukey", "rfcva": "RFCVA Cauchy"}


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results/research/va_bakeoff_20260720")
    ds = sys.argv[2] if len(sys.argv) > 2 else "deep"
    rows = []
    for tag in ORDER:
        p = out / tag / ds / "metrics.json"
        if not p.is_file():
            rows.append((tag, None)); continue
        rows.append((tag, json.load(p.open())["metrics"]))

    bl = next((m for t, m in rows if t == "bl" and m), None)
    print(f"\n=== VA BAKE-OFF — {ds} ({out}) ===")
    print(f"{'method':13s} {'h(m)':>7s} {'u(m)':>7s} {'yaw°':>6s} {'fix%':>6s} {'matched':>8s}  vs-bl Δh")
    table = []
    for tag, m in rows:
        if not m:
            print(f"{LABEL[tag]:13s}  (no result / failed)"); continue
        dh = (m["rmse_h_m"] - bl["rmse_h_m"]) if bl else 0.0
        table.append((tag, m, dh))
        print(f"{LABEL[tag]:13s} {m['rmse_h_m']:7.3f} {m.get('rmse_u_m',float('nan')):7.3f} "
              f"{(m.get('yaw_rmse_deg') or float('nan')):6.2f} {100*m['fixed_rate']:6.1f} "
              f"{m['n_matched']:8d}  {dh:+.3f}")

    done = [(t, m) for t, m in rows if m]
    if done:
        best_h = min(done, key=lambda x: x[1]["rmse_h_m"])
        best_u = min(done, key=lambda x: x[1].get("rmse_u_m", 9e9))
        print(f"\nBest horizontal RMSE : {LABEL[best_h[0]]}  ({best_h[1]['rmse_h_m']:.3f} m)")
        print(f"Best vertical RMSE   : {LABEL[best_u[0]]}  ({best_u[1].get('rmse_u_m',float('nan')):.3f} m)")
        print("NOTE: n=1 per method here (screening). Confirm the winner with n=3 before the paper.")
    ranking = {"dataset": ds, "methods": {t: m for t, m in rows}}
    (out / "bakeoff_ranking.json").write_text(json.dumps(ranking, indent=2, default=str) + "\n")
    print(f"\nWrote {out/'bakeoff_ranking.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
