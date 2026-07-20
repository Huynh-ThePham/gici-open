#!/usr/bin/env python3
"""Group-1 spike (classical GNSS-only multipath mitigation ceiling): measure whether
code-multipath (MP combination) and SNR carry ANY discriminating power at the
high-horizontal-error epochs on Deep. Same logic as M1 (spike_m1_los_nlos.py): GT is
used ONLY to label high-error epochs, never in any estimator.

If MP/SNR do NOT separate high-error from low-error epochs (like NLOS count, corr~0),
then no SNR/CMC/Hatch WEIGHTING scheme can remove the tail -> ceiling ~0, no point
building the estimator variant. If they DO, there is headroom worth an implementation.

Multipath observable = Estey-Meertens MP combination (dual-freq, iono-free):
    MP1 = C1 - (1 + 2/(a-1)) * Phi1 + (2/(a-1)) * Phi2 ,  a = (f1/f2)^2
per continuous arc MP1 = code_multipath + ambiguity_const -> subtract arc mean.
Arcs split on time gap > 30 s OR |MP jump| > 4 m (cycle slip).
"""
from __future__ import annotations
import math, sys, warnings
from pathlib import Path
warnings.filterwarnings("ignore")
import numpy as np
import georinex as gr

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_urbannav_rrr_baseline import (load_ground_truth, parse_solution, interp_llh,
    llh_to_ecef, ecef_to_enu, DATASETS)

ROOT = DATASETS["deep"].root
OBS = ROOT / DATASETS["deep"].rover
SOL = Path("results/research/va_bakeoff_iso_20260720/bl/deep/output/solution.txt")
GPS_WEEK_DAY_OFFSET = DATASETS["deep"].gps_week_day_offset
C = 299792458.0
HIGH_ERR_M = 5.0
LOW_ERR_M = 2.0
MP_HI_M = 1.0        # code-multipath "significant" threshold (m)
SNR_LO_DBHZ = 35.0   # low-SNR threshold

# per-system: (code1, phase1, snr1, code2, phase2, f1, f2)
SYS = {
    "G": ("C1C", "L1C", "S1C", "C2L", "L2L", 1575.42e6, 1227.60e6),
    "E": ("C1C", "L1C", "S1C", "C7Q", "L7Q", 1575.42e6, 1207.140e6),
    "C": ("C2I", "L2I", "S2I", "C7I", "L7I", 1561.098e6, 1207.140e6),
}


def mp_coeffs(f1, f2):
    a = (f1 / f2) ** 2
    return 1.0 + 2.0 / (a - 1.0), 2.0 / (a - 1.0)


def gpst_of_obs(t_np):
    ts = (np.datetime64(t_np) - np.datetime64("1980-01-06T00:00:00")) / np.timedelta64(1, "s")
    return (ts + 18.0) % 604800  # GPS-UTC leap = 18 (2021)


def main():
    meas = []
    for s in SYS.values():
        meas += [s[0], s[1], s[2], s[3], s[4]]
    meas = sorted(set(meas))
    print(f"loading obs (vars={meas}) ...", flush=True)
    obs = gr.load(str(OBS), meas=meas, useindicators=False)
    tvals = obs.time.values
    svs = [str(s) for s in obs.sv.values]
    ne = len(tvals)
    print(f"  epochs={ne} sats={len(svs)}")
    tow = np.array([gpst_of_obs(t) for t in tvals])

    dv = set(obs.data_vars)
    # Build per-sat MP1 time series -> detrended code multipath; and SNR series.
    # mp_ep[ei] = list of |mp| for sats present; snr_ep[ei] = list of snr.
    mp_ep = [[] for _ in range(ne)]
    snr_ep = [[] for _ in range(ne)]
    n_used_sat = 0
    for si, sv in enumerate(svs):
        sysc = sv[0]
        if sysc not in SYS:
            continue
        c1n, l1n, s1n, c2n, l2n, f1, f2 = SYS[sysc]
        if not all(v in dv for v in (c1n, l1n, c2n, l2n)):
            continue
        lam1, lam2 = C / f1, C / f2
        a, b = mp_coeffs(f1, f2)
        c1 = obs[c1n].values[:, si]
        l1 = obs[l1n].values[:, si]
        c2 = obs[c2n].values[:, si]
        l2 = obs[l2n].values[:, si]
        snr = obs[s1n].values[:, si] if s1n in dv else np.full(ne, np.nan)
        phi1 = l1 * lam1
        phi2 = l2 * lam2
        mp = c1 - a * phi1 + b * phi2   # MP1, meters (nan where any missing)
        valid = np.isfinite(mp)
        if valid.sum() < 5:
            continue
        n_used_sat += 1
        # split into arcs on time gap or MP jump; subtract arc mean
        idx = np.where(valid)[0]
        arc = [idx[0]]
        arcs = []
        for k in range(1, len(idx)):
            i0, i1 = idx[k - 1], idx[k]
            dt = tow[i1] - tow[i0]
            if dt < 0:
                dt += 604800
            jump = abs(mp[i1] - mp[i0])
            if dt > 30.0 or jump > 4.0:
                arcs.append(arc); arc = [i1]
            else:
                arc.append(i1)
        arcs.append(arc)
        for arc in arcs:
            if len(arc) < 4:
                continue
            m = mp[arc]
            mres = m - np.mean(m)   # remove ambiguity/bias const (iono already gone)
            for j, ei in enumerate(arc):
                mp_ep[ei].append(abs(mres[j]))
                sv_snr = snr[ei]
                if np.isfinite(sv_snr) and sv_snr > 0:
                    snr_ep[ei].append(sv_snr)
    print(f"  sats used for MP: {n_used_sat}")

    # per-epoch aggregates
    print("loading solution + GT for h_err labels ...", flush=True)
    gt = load_ground_truth(DATASETS["deep"].root / DATASETS["deep"].gt_file)
    sol = parse_solution(SOL, GPS_WEEK_DAY_OFFSET)
    import bisect
    sol_t = [s["gpst"] for s in sol]
    ref0 = llh_to_ecef(*gt["llh"][0]); r0lat, r0lon = gt["llh"][0][0], gt["llh"][0][1]

    rows = []  # (h_err, mp_max, mp_mean, mp_p90, n_mp_hi, snr_min, snr_mean, n_snr_lo, nsat)
    for ei in range(ne):
        mps = mp_ep[ei]
        if len(mps) < 4:
            continue
        gpst_full = tow[ei]
        j = bisect.bisect_left(sol_t, gpst_full)
        cand = [k for k in (j - 1, j) if 0 <= k < len(sol_t)]
        if not cand:
            continue
        k = min(cand, key=lambda k: abs(sol_t[k] - gpst_full))
        if abs(sol_t[k] - gpst_full) > 2.0:
            continue
        gp = interp_llh(gt["times"], gt["llh"], gpst_full)
        if gp is None:
            continue
        se = ecef_to_enu(*llh_to_ecef(sol[k]["lat"], sol[k]["lon"], sol[k]["h"]), ref0, r0lat, r0lon)
        ge = ecef_to_enu(*llh_to_ecef(*gp), ref0, r0lat, r0lon)
        h_err = math.hypot(se[0] - ge[0], se[1] - ge[1])
        mps = np.array(mps)
        snrs = np.array(snr_ep[ei]) if snr_ep[ei] else np.array([np.nan])
        rows.append((h_err, mps.max(), mps.mean(), np.percentile(mps, 90),
                     int((mps > MP_HI_M).sum()),
                     np.nanmin(snrs), np.nanmean(snrs),
                     int((snrs < SNR_LO_DBHZ).sum()), len(mps)))

    rows = np.array(rows)
    print(f"\n=== G1 MP/SNR discriminating power on Deep — {len(rows)} epochs "
          f"(GEC dual-freq MP combination; GT only labels h_err) ===")
    h = rows[:, 0]
    names = ["mp_max", "mp_mean", "mp_p90", "n_mp>%.1fm" % MP_HI_M,
             "snr_min", "snr_mean", "n_snr<%.0f" % SNR_LO_DBHZ]
    cols = [1, 2, 3, 4, 5, 6, 7]

    def pearson(x, y):
        m = np.isfinite(x) & np.isfinite(y)
        if m.sum() < 10:
            return float("nan")
        return float(np.corrcoef(x[m], y[m])[0, 1])

    print(f"\n  corr(h_err, indicator)   [|corr|>~0.3 = real headroom; ~0 = no ceiling]")
    for name, c in zip(names, cols):
        print(f"    {name:12s}: {pearson(rows[:, c], h):+.3f}")

    hi = rows[h > HIGH_ERR_M]
    lo = rows[h <= LOW_ERR_M]
    print(f"\n  high-err epochs (h>{HIGH_ERR_M}m): n={len(hi)}   low-err (h<={LOW_ERR_M}m): n={len(lo)}")
    print(f"  {'indicator':12s} {'HIGH-err med':>14s} {'LOW-err med':>14s} {'ratio':>8s}")
    for name, c in zip(names, cols):
        mh = np.nanmedian(hi[:, c]) if len(hi) else float("nan")
        ml = np.nanmedian(lo[:, c]) if len(lo) else float("nan")
        ratio = mh / ml if ml else float("nan")
        print(f"  {name:12s} {mh:14.2f} {ml:14.2f} {ratio:8.2f}")

    import json
    Path("results/research").mkdir(parents=True, exist_ok=True)
    Path("results/research/spike_g1_deep.json").write_text(json.dumps({
        "n_epochs": len(rows), "high_err_m": HIGH_ERR_M,
        "corr": {name: pearson(rows[:, c], h) for name, c in zip(names, cols)},
        "high_med": {name: (float(np.nanmedian(hi[:, c])) if len(hi) else None) for name, c in zip(names, cols)},
        "low_med": {name: (float(np.nanmedian(lo[:, c])) if len(lo) else None) for name, c in zip(names, cols)},
    }, indent=2))
    print("\nwrote results/research/spike_g1_deep.json")
    print("\nDECISION: if all |corr|<~0.2 and HIGH/LOW ratios ~1.0 -> MP & SNR are BLIND to")
    print("  the high-error epochs -> classical SNR/CMC/Hatch weighting CANNOT remove the")
    print("  tail (same verdict as M1 for NLOS). Any |corr|>~0.3 or ratio>>1 = real headroom.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
