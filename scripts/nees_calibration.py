#!/usr/bin/env python3
"""NEES + calibration test of GICI's reported position covariance vs ground truth.

Tests the INTEGRITY hypothesis: is the estimator's per-epoch position uncertainty
($GPESD STD_Pe/Pn/Pu, from solution.covariance with compute_covariance:true)
statistically consistent with the actual error vs ground truth?

  NEES_3D = (e/σe)^2 + (n/σn)^2 + (u/σu)^2 ;  consistent => E[NEES]=3 (chi-sq_3).
  Overconfident (σ too small) => NEES >> 3.  Conservative => NEES < 3.
  Calibration: fraction of epochs with |axis err| < k·σ vs Gaussian 68/95/99.7%.

GT is used ONLY here in evaluation (allowed). Usage:
  python3 scripts/nees_calibration.py <solution.txt> <ground_truth.txt> [--std-max M]
"""
from __future__ import annotations
import sys, math, argparse, bisect
import numpy as np

WGS84_A = 6378137.0; WGS84_F = 1/298.257223563; WGS84_E2 = WGS84_F*(2-WGS84_F)
LEAP = 18.0


def llh2ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    n = WGS84_A/math.sqrt(1-WGS84_E2*math.sin(la)**2)
    return ((n+h)*math.cos(la)*math.cos(lo), (n+h)*math.cos(la)*math.sin(lo),
            (n*(1-WGS84_E2)+h)*math.sin(la))


def ecef2enu(x, y, z, ref, lat0, lon0):
    la, lo = math.radians(lat0), math.radians(lon0)
    dx, dy, dz = x-ref[0], y-ref[1], z-ref[2]
    sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    return (-so*dx+co*dy, -sl*co*dx-sl*so*dy+cl*dz, cl*co*dx+cl*so*dy+sl*dz)


def load_gt(path):
    # Week GPSTime UTCTime Longitude Latitude H-Ell Heading Pitch ...
    t, llh = [], []
    for line in open(path, errors="replace"):
        p = line.split()
        if len(p) < 6 or not p[0].isdigit():
            continue
        try:
            utc = float(p[2]); lon = float(p[3]); lat = float(p[4]); h = float(p[5])
        except ValueError:
            continue
        t.append(utc); llh.append((lat, lon, h))
    return t, llh


def interp_llh(ts, llh, x):
    if x < ts[0] or x > ts[-1]:
        return None
    i = bisect.bisect_left(ts, x); i = min(max(i, 1), len(ts)-1)
    t0, t1 = ts[i-1], ts[i]
    a = 0 if t1 <= t0 else (x-t0)/(t1-t0)
    return tuple(llh[i-1][k]+a*(llh[i][k]-llh[i-1][k]) for k in range(3))


def tod(nmea_time):  # HHMMSS.sss -> seconds of day
    return int(nmea_time[:2])*3600 + int(nmea_time[2:4])*60 + float(nmea_time[4:])


def dm2deg(tok, hemi, is_lat):
    d = int(tok[:2] if is_lat else tok[:3]); m = float(tok[2:] if is_lat else tok[3:])
    v = d + m/60
    return -v if hemi in ("S", "W") else v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("solution"); ap.add_argument("gt")
    ap.add_argument("--std-max", type=float, default=5.0,
                    help="drop epochs whose reported STD exceeds this (m): init/rank-deficient")
    ap.add_argument("--day-offset", type=float, default=None)
    a = ap.parse_args()

    gt_t, gt_llh = load_gt(a.gt)
    # day offset: GT UTCTime is ToW-scale; GPGGA tod is seconds-of-day
    off = a.day_offset if a.day_offset is not None else (gt_t[0]//86400)*86400

    gga, esd = {}, {}
    for line in open(a.solution, errors="replace"):
        if line.startswith("$GPGGA"):
            c = line.split(",")
            try:
                key = c[1]
                lat = dm2deg(c[2], c[3], True); lon = dm2deg(c[4], c[5], False)
                h = float(c[9]) + (float(c[11]) if c[11] else 0.0)
                gga[key] = (lat, lon, h)
            except (ValueError, IndexError):
                pass
        elif line.startswith("$GPESD"):
            c = line.split(",")
            try:
                key = c[1]
                se, sn, su = float(c[2]), float(c[3]), float(c[4])
                esd[key] = (se, sn, su)
            except (ValueError, IndexError):
                pass

    ref = llh2ecef(*gt_llh[0])
    nees, ze, zn, zu = [], [], [], []
    n_drop_std = n_nogt = 0
    for key in gga:
        if key not in esd:
            continue
        se, sn, su = esd[key]
        if max(se, sn, su) > a.std_max or min(se, sn, su) <= 0:
            n_drop_std += 1; continue
        t_utc = off + tod(key)
        gp = interp_llh(gt_t, gt_llh, t_utc)
        if gp is None:
            n_nogt += 1; continue
        se_ecef = llh2ecef(*gga[key]); ge_ecef = llh2ecef(*gp)
        s_enu = ecef2enu(*se_ecef, ref, gt_llh[0][0], gt_llh[0][1])
        g_enu = ecef2enu(*ge_ecef, ref, gt_llh[0][0], gt_llh[0][1])
        e, n, u = s_enu[0]-g_enu[0], s_enu[1]-g_enu[1], s_enu[2]-g_enu[2]
        ze.append(e/se); zn.append(n/sn); zu.append(u/su)
        nees.append((e/se)**2 + (n/sn)**2 + (u/su)**2)

    nees = np.array(nees); ze = np.array(ze); zn = np.array(zn); zu = np.array(zu)
    N = len(nees)
    if N == 0:
        print("No valid epochs (all dropped)."); return 1
    anees = nees.mean()
    # chi-square_3 95% single-epoch bounds ~ [0.216, 9.35]; ANEES 95% CI ~ 3 ± 3*1.96*sqrt(2*3/N)
    ci = 3 * 1.96 * math.sqrt(2*3/N)
    print(f"\n=== NEES / CALIBRATION — {a.solution} ===")
    print(f"valid epochs N={N} (dropped std>{a.std_max}m: {n_drop_std}, no-GT: {n_nogt})")
    print(f"ANEES = {anees:.3f}   (consistent target = 3.0 ± {ci:.2f} @95%)")
    verdict = ("CONSISTENT" if abs(anees-3) <= ci else
               "OVERCONFIDENT (σ too small)" if anees > 3 else "CONSERVATIVE (σ too large)")
    print(f"  -> {verdict}")
    print(f"per-axis E[z^2] (target 1.0): E={((ze**2).mean()):.2f}  N={((zn**2).mean()):.2f}  U={((zu**2).mean()):.2f}")
    print("calibration coverage (target 68.3 / 95.4 / 99.7 %):")
    for lab, z in (("E", ze), ("N", zn), ("U", zu)):
        c1 = 100*np.mean(np.abs(z) < 1); c2 = 100*np.mean(np.abs(z) < 2); c3 = 100*np.mean(np.abs(z) < 3)
        print(f"  {lab}: 1σ={c1:.0f}%  2σ={c2:.0f}%  3σ={c3:.0f}%   (RMS err/σ = {math.sqrt((z**2).mean()):.2f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
