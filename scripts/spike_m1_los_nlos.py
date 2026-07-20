#!/usr/bin/env python3
"""M1 feasibility spike (vision-NLOS): count LOS vs NLOS satellites at Deep epochs,
especially the high horizontal-error ones, to decide whether NLOS EXCLUSION can
recover horizontal accuracy (enough LOS left?) or whether the tail is majority-NLOS
(exclusion would starve -> need NLOS correction, not exclusion).

Method (GT used only to *label* high-error epochs; not in any estimator):
  - receiver position per epoch : Deep baseline ISO solution.txt (GPGGA).
  - observed satellites/epoch   : rover RINEX obs (georinex).
  - satellite ECEF              : broadcast ephemeris (Kepler, IS-GPS-200 form for
                                  G/E/C; GLONASS R excluded per config, SBAS S skipped).
  - LOS/NLOS                    : skymask skyline at (nearest grid pt, round(az));
                                  NLOS iff sat_elev < skyline_elev.
  - high-error epochs           : |horizontal error vs GT| above a threshold.

Outputs a decision table: at high-error epochs, median #LOS after exclusion + how
often it drops below a usable count (>=5). Approx az/el (~1 deg) is fine for skyline.
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
NAV = ROOT / "gnss/base/_deprecated/brdc_mn.rnx"
OBS = ROOT / DATASETS["deep"].rover
SKYMASK = ROOT / "UrbanNav-HK-Deep-Urban-WP.csv"
SOL = Path("results/research/va_bakeoff_iso_20260720/bl/deep/output/solution.txt")
GPS_WEEK_DAY_OFFSET = DATASETS["deep"].gps_week_day_offset
MU = 3.986005e14
OMGE = 7.2921151467e-5
HIGH_ERR_M = 5.0          # epoch is "high-error" if horizontal err > this
MIN_ELEV = 7.0            # GICI min_elevation mask
USABLE_LOS = 5            # need >= this many LOS sats for a usable fix geometry


def kepler_ecef(eph, t_tow):
    """Broadcast-ephemeris -> ECEF (m). eph: dict of RINEX nav fields. t_tow: GPS ToW (s)."""
    A = eph["sqrtA"] ** 2
    n0 = math.sqrt(MU / A**3)
    tk = t_tow - eph["Toe"]
    if tk > 302400: tk -= 604800
    if tk < -302400: tk += 604800
    n = n0 + eph["DeltaN"]
    M = eph["M0"] + n * tk
    E = M
    for _ in range(15):
        E = M + eph["Eccentricity"] * math.sin(E)
    e = eph["Eccentricity"]
    v = math.atan2(math.sqrt(1 - e*e) * math.sin(E), math.cos(E) - e)
    phi = v + eph["omega"]
    du = eph["Cus"]*math.sin(2*phi) + eph["Cuc"]*math.cos(2*phi)
    dr = eph["Crs"]*math.sin(2*phi) + eph["Crc"]*math.cos(2*phi)
    di = eph["Cis"]*math.sin(2*phi) + eph["Cic"]*math.cos(2*phi)
    u = phi + du
    r = A*(1 - e*math.cos(E)) + dr
    i = eph["Io"] + di + eph["IDOT"]*tk
    xp = r*math.cos(u); yp = r*math.sin(u)
    Omg = eph["Omega0"] + (eph["OmegaDot"] - OMGE)*tk - OMGE*eph["Toe"]
    x = xp*math.cos(Omg) - yp*math.cos(i)*math.sin(Omg)
    y = xp*math.sin(Omg) + yp*math.cos(i)*math.cos(Omg)
    z = yp*math.sin(i)
    return x, y, z


def az_el(rx_ecef, rx_lat, rx_lon, sat_ecef):
    e, n, u = ecef_to_enu(*sat_ecef, rx_ecef, rx_lat, rx_lon)
    az = (math.degrees(math.atan2(e, n))) % 360.0
    el = math.degrees(math.atan2(u, math.hypot(e, n)))
    return az, el


def load_ephemeris(nav_path):
    nav = gr.load(str(nav_path))
    field = {"DeltaN": "DeltaN", "M0": "M0", "Cuc": "Cuc", "Cus": "Cus", "Crc": "Crc",
             "Crs": "Crs", "Cic": "Cic", "Cis": "Cis", "Eccentricity": "Eccentricity",
             "sqrtA": "sqrtA", "Toe": "Toe", "Io": "Io", "omega": "omega",
             "Omega0": "Omega0", "OmegaDot": "OmegaDot", "IDOT": "IDOT"}
    out = {}  # prn -> list of (toe_seconds, ephdict)
    times = nav.time.values
    for sv in nav.sv.values:
        prn = str(sv)
        if prn[0] not in "GEC":     # keep GPS/Galileo/BeiDou; skip R (excluded), S
            continue
        sub = nav.sel(sv=sv)
        for ti in range(len(times)):
            try:
                toe = float(sub["Toe"].values[ti])
            except Exception:
                continue
            if math.isnan(toe):
                continue
            ed = {}
            ok = True
            for k, v in field.items():
                try:
                    val = float(sub[v].values[ti])
                except Exception:
                    ok = False; break
                if math.isnan(val): ok = False; break
                ed[k] = val
            if not ok:
                continue
            out.setdefault(prn, []).append((toe, ed))
    return out


def best_eph(eph_list, tow):
    return min(eph_list, key=lambda te: abs(((tow - te[0] + 302400) % 604800) - 302400))[1]


def build_skymask(path):
    lats, lons, masks = [], [], []
    with path.open() as f:
        f.readline()
        for line in f:
            p = line.split(",")
            if len(p) != 364: continue
            lats.append(float(p[0])); lons.append(float(p[1]))
            masks.append(np.array(p[3:], dtype=np.float32))
    return np.array(lats), np.array(lons), np.array(masks)


def main():
    print("loading ephemeris...", flush=True)
    ephs = load_ephemeris(NAV)
    print(f"  {len(ephs)} sats with ephemeris ({sorted(set(p[0] for p in ephs))})")
    print("loading skymask...", flush=True)
    sm_lat, sm_lon, sm_mask = build_skymask(SKYMASK)
    print(f"  {len(sm_lat)} grid pts")
    print("loading rover obs (sats per epoch)...", flush=True)
    obs = gr.load(str(OBS), meas=["C1C"] if False else None, useindicators=False)
    obs_times = obs.time.values
    obs_sv = [str(s) for s in obs.sv.values]
    # per epoch: which sats have a pseudorange (any C code) -> use C1C/C1X presence
    # georinex: a sat is "observed" if any obs var is finite at that time
    print(f"  obs epochs={len(obs_times)} sats={len(obs_sv)}")
    print("loading receiver solution + GT...", flush=True)
    gt = load_ground_truth(DATASETS["deep"].gt_file and DATASETS["deep"].root / DATASETS["deep"].gt_file)
    sol = parse_solution(SOL, GPS_WEEK_DAY_OFFSET)
    # index solution by gpst for receiver pos + error
    import bisect
    sol_t = [s["gpst"] for s in sol]

    # pick an obs variable to test "observed" (first pseudorange-like var)
    pr_vars = [v for v in obs.data_vars if v.startswith("C")]
    print(f"  pseudorange vars: {pr_vars[:6]}")

    def gpst_of_obs(t_np):
        # obs time is UTC datetime64 -> GPS ToW. Deep DOY141 week2158. Use unix->gpst.
        ts = (np.datetime64(t_np) - np.datetime64("1980-01-06T00:00:00")) / np.timedelta64(1, "s")
        # leap seconds GPS-UTC = 18 (2021)
        return (ts + 18.0) % 604800

    rows = []  # per epoch: (h_err, n_obs, n_los, n_nlos, gdop_ok)
    ne = len(obs_times)
    step = max(1, ne // 400)   # sample ~400 epochs across the run
    for ei in range(0, ne, step):
        t_np = obs_times[ei]
        tow = gpst_of_obs(t_np)
        # full gpst for solution lookup = offset day + tow-in-day; use tow directly vs sol gpst
        # sol gpst uses same week ToW scale (offset+seconds). Match by nearest.
        gpst_full = tow
        # receiver pos: nearest solution epoch (sol gpst are offset+tod+leap ~ same ToW)
        j = bisect.bisect_left(sol_t, gpst_full)
        cand = [k for k in (j-1, j) if 0 <= k < len(sol_t)]
        if not cand: continue
        k = min(cand, key=lambda k: abs(sol_t[k]-gpst_full))
        if abs(sol_t[k]-gpst_full) > 2.0: continue
        # horizontal error of the SOLUTION vs GT (labels the epoch)
        gp = interp_llh(gt["times"], gt["llh"], gpst_full)
        if gp is None: continue
        sol_ecef = llh_to_ecef(sol[k]["lat"], sol[k]["lon"], sol[k]["h"])
        ref0 = llh_to_ecef(*gt["llh"][0]); r0lat, r0lon = gt["llh"][0][0], gt["llh"][0][1]
        se = ecef_to_enu(*sol_ecef, ref0, r0lat, r0lon)
        ge = ecef_to_enu(*llh_to_ecef(*gp), ref0, r0lat, r0lon)
        h_err = math.hypot(se[0]-ge[0], se[1]-ge[1])
        # GEOMETRY uses the TRUE (GT) receiver position, not the erroneous solution:
        # at high-error epochs the solution is off by up to ~17 m, which would query the
        # skymask at the wrong grid cell and corrupt exactly the epochs we care about.
        rx = {"lat": gp[0], "lon": gp[1], "h": gp[2]}
        rx_ecef = llh_to_ecef(rx["lat"], rx["lon"], rx["h"])
        # nearest skymask grid point (to TRUE position)
        gi = int(np.argmin((sm_lat-rx["lat"])**2 + (sm_lon-rx["lon"])**2))
        skyline = sm_mask[gi]
        # observed sats this epoch
        n_los = n_nlos = n_obs = n_nlos_flip = 0
        for si, sv in enumerate(obs_sv):
            if sv[0] not in "GEC": continue
            if sv not in ephs: continue
            # observed? any pseudorange finite
            observed = False
            for pv in pr_vars:
                try:
                    val = float(obs[pv].values[ei, si])
                    if not math.isnan(val) and val != 0.0: observed = True; break
                except Exception: continue
            if not observed: continue
            eph = best_eph(ephs[sv], tow)
            try:
                sat = kepler_ecef(eph, tow)
            except Exception: continue
            az, el = az_el(rx_ecef, rx["lat"], rx["lon"], sat)
            if el < MIN_ELEV: continue    # below mask, not used anyway
            n_obs += 1
            skyl = skyline[int(round(az)) % 361]
            skyl_flip = skyline[int(round(360.0 - az)) % 361]   # azimuth-convention check
            if el >= skyl: n_los += 1
            else: n_nlos += 1
            if el < skyl_flip: n_nlos_flip += 1
        rows.append((gpst_full, h_err, n_obs, n_los, n_nlos, n_nlos_flip))

    rows = [r for r in rows if r[2] >= 4]
    print(f"\n=== M1 LOS/NLOS on Deep — {len(rows)} sampled epochs (elev>{MIN_ELEV}, TRUE/GT position) ===")
    # azimuth-convention sanity: total NLOS under normal vs flipped az
    tot_n = sum(r[4] for r in rows); tot_nflip = sum(r[5] for r in rows); tot = sum(r[2] for r in rows)
    print(f"  [az-convention check] NLOS%% normal={100*tot_n/max(1,tot):.0f}%  flipped(360-az)={100*tot_nflip/max(1,tot):.0f}%  "
          f"(large gap => convention matters; pick the one giving MORE structure vs error)")
    def summ(label, subset):
        if not subset:
            print(f"  {label}: (none)"); return
        obsv = np.array([r[2] for r in subset]); los = np.array([r[3] for r in subset])
        nlos = np.array([r[4] for r in subset])
        frac_nlos = nlos.sum()/max(1,(los.sum()+nlos.sum()))
        starved = np.mean(los < USABLE_LOS)*100
        print(f"  {label} (n={len(subset)}): obs med={np.median(obsv):.0f}  "
              f"LOS med={np.median(los):.0f}  NLOS med={np.median(nlos):.0f}  "
              f"NLOS%={100*frac_nlos:.0f}%  epochs<{USABLE_LOS}LOS={starved:.0f}%")
    summ("ALL", rows)
    summ(f"HIGH-ERR (h>{HIGH_ERR_M}m)", [r for r in rows if r[1] > HIGH_ERR_M])
    summ("LOW-ERR (h<=2m)", [r for r in rows if r[1] <= 2.0])
    # decision
    hi = [r for r in rows if r[1] > HIGH_ERR_M]
    if hi:
        los_hi = np.array([r[3] for r in hi])
        starved = np.mean(los_hi < USABLE_LOS)*100
        print(f"\nDECISION: at high-error epochs, after NLOS exclusion median LOS={np.median(los_hi):.0f}; "
              f"{starved:.0f}% of them fall below {USABLE_LOS} LOS.")
        print("  -> MERGE-viable if median LOS >= ~6 and starved% low; "
              "else majority-NLOS -> exclusion starves -> need NLOS correction (SPLIT).")
    import json
    Path("results/research/spike_m1_deep.json").write_text(json.dumps(
        {"n_epochs": len(rows), "high_err_m": HIGH_ERR_M,
         "rows": [{"gpst": r[0], "h_err": r[1], "n_obs": r[2], "los": r[3], "nlos": r[4],
                   "nlos_flip": r[5]} for r in rows]},
        indent=2))
    print("\nWrote results/research/spike_m1_deep.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
