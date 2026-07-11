#!/usr/bin/env python3
"""Run author UrbanNav RTK/IMU/Camera RRR baseline (f2b8579 templates only).

Uses research/config/rtk_imu_camera_rrr_urbannav.yaml derived from:
  - option/post_estimation_RTK_RRR_rinex_imutext.yaml
  - ros_wrapper/.../ros_urbannav.yaml RRR block

Does NOT use UrbanNavDataset/gici_rrr/config.yaml or any paper1 fork.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[1]
GICI_MAIN = Path(
    os.environ.get(
        "GICI_MAIN",
        str(REPO / "build" / "gici_main"),
    )
)
TEMPLATE = REPO / "research" / "config" / "rtk_imu_camera_rrr_urbannav.yaml"
DATA_ROOT = Path("/home/theph/Downloads/UrbanNavDataset-master")
GPS_UTC_LEAP_SECONDS = 18.0
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

PAPER_APE = {
    "medium": {"pos_m": 3.40, "rot_deg": 1.30},
    "deep": {"pos_m": 2.46, "rot_deg": 1.64},
}


@dataclass(frozen=True)
class Dataset:
    key: str
    title: str
    root: Path
    gt_file: str
    gps_week_day_offset: float
    rover: str
    base: str
    eph: str
    dcb: str
    timeout_s: int


DATASETS: dict[str, Dataset] = {
    "medium": Dataset(
        key="medium",
        title="UrbanNav-HK-Medium-Urban-1",
        root=DATA_ROOT / "UrbanNav-HK-Medium-Urban-1",
        gt_file="UrbanNav_TST_GT_raw.txt",
        gps_week_day_offset=86400.0,
        rover="gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs",
        base="gnss/base/hkkt137g.rnx",
        eph="gnss/base/brdc1370.rnx",
        dcb="research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX",
        timeout_s=7200,
    ),
    "deep": Dataset(
        key="deep",
        title="UrbanNav-HK-Deep-Urban-1",
        root=DATA_ROOT / "UrbanNav-HK-Deep-Urban-1",
        gt_file="UrbanNav_whampoa_raw.txt",
        gps_week_day_offset=432000.0,
        rover="gnss/UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs",
        # Author GICI paper eval uses session-aligned 5 s base (not full-day HKKT).
        base="gnss/base/_deprecated/hkkt141g.21o",
        eph="gnss/base/_deprecated/brdc_mn.rnx",
        dcb="research/dcb/CAS0MGXRAP_20211410000_01D_01D_DCB.BSX",
        timeout_s=10800,
    ),
}


def dms_to_deg(d: float, m: float, s: float) -> float:
    sign = -1.0 if d < 0 else 1.0
    return sign * (abs(d) + m / 60.0 + s / 3600.0)


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
    ref: tuple[float, float, float],
    lat0_deg: float, lon0_deg: float,
) -> tuple[float, float, float]:
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    dx, dy, dz = x - ref[0], y - ref[1], z - ref[2]
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u


def parse_nmea_time(token: str) -> float:
    return int(token[0:2]) * 3600.0 + int(token[2:4]) * 60.0 + float(token[4:])


def dm_to_deg(token: str, hemi: str, is_lat: bool) -> float:
    deg = int(token[0:2] if is_lat else token[0:3])
    minutes = float(token[2:] if is_lat else token[3:])
    out = deg + minutes / 60.0
    return -out if hemi in ("S", "W") else out


def angle_error_deg(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def interp_scalar(times: list[float], values: list[float], t: float, angular: bool = False) -> float | None:
    if t < times[0] or t > times[-1]:
        return None
    lo, hi = 0, len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = times[lo], times[hi]
    if t1 <= t0:
        return values[lo]
    alpha = (t - t0) / (t1 - t0)
    if angular:
        return values[lo] + alpha * angle_error_deg(values[hi], values[lo])
    return values[lo] + alpha * (values[hi] - values[lo])


def interp_llh(times: list[float], llh: list[tuple[float, float, float]], t: float):
    if t < times[0] or t > times[-1]:
        return None
    lo, hi = 0, len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = times[lo], times[hi]
    if t1 <= t0:
        return llh[lo]
    alpha = (t - t0) / (t1 - t0)
    lat = llh[lo][0] + alpha * (llh[hi][0] - llh[lo][0])
    lon = llh[lo][1] + alpha * (llh[hi][1] - llh[lo][1])
    h = llh[lo][2] + alpha * (llh[hi][2] - llh[lo][2])
    return lat, lon, h


def load_ground_truth(path: Path) -> dict[str, list[Any]]:
    times: list[float] = []
    llh: list[tuple[float, float, float]] = []
    heading: list[float] = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith(("UTCTime", "(sec)", "GPSTime")):
            continue
        parts = line.split()
        if len(parts) < 20:
            continue
        try:
            gpst = float(parts[2])
            lat = dms_to_deg(float(parts[3]), float(parts[4]), float(parts[5]))
            lon = dms_to_deg(float(parts[6]), float(parts[7]), float(parts[8]))
            h = float(parts[9])
            hdg = float(parts[18]) % 360.0
        except ValueError:
            continue
        times.append(gpst)
        llh.append((lat, lon, h))
        heading.append(hdg)
    if not times:
        raise RuntimeError(f"No ground truth parsed from {path}")
    return {"times": times, "llh": llh, "heading": heading}


def parse_solution(path: Path, gps_week_day_offset: float) -> list[dict[str, Any]]:
    esa_by_time: dict[str, dict[str, float]] = {}
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("$GPESA,"):
            continue
        parts = line.split("*", 1)[0].split(",")
        if len(parts) < 8:
            continue
        try:
            esa_by_time[parts[1]] = {
                "yaw": float(parts[7]) % 360.0,
            }
        except ValueError:
            continue

    epochs: list[dict[str, Any]] = []
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("$GPGGA,"):
            continue
        parts = line.split("*", 1)[0].split(",")
        if len(parts) < 12:
            continue
        try:
            token = parts[1]
            lat = dm_to_deg(parts[2], parts[3], True)
            lon = dm_to_deg(parts[4], parts[5], False)
            quality = int(parts[6])
            alt_msl = float(parts[9])
            geoid_sep = float(parts[11]) if parts[11] else 0.0
        except (ValueError, IndexError):
            continue
        entry = {
            "gpst": gps_week_day_offset + parse_nmea_time(token) + GPS_UTC_LEAP_SECONDS,
            "lat": lat,
            "lon": lon,
            "h": alt_msl + geoid_sep,
            "quality": quality,
        }
        entry.update(esa_by_time.get(token, {}))
        epochs.append(entry)
    return epochs


def evaluate_solution(solution: Path, gt: dict[str, list[Any]], gps_week_day_offset: float) -> dict[str, Any]:
    epochs = parse_solution(solution, gps_week_day_offset)
    if not epochs:
        raise RuntimeError(f"No GPGGA epochs in {solution}")

    gt_times = gt["times"]
    gt_llh = gt["llh"]
    ref_lat, ref_lon, ref_h = gt_llh[0]
    ref_ecef = llh_to_ecef(ref_lat, ref_lon, ref_h)

    e_vals, n_vals, u_vals, h_vals, yaw_err_vals = [], [], [], [], []
    quality_counts: dict[int, int] = {}

    for ep in epochs:
        quality_counts[ep["quality"]] = quality_counts.get(ep["quality"], 0) + 1
        gt_pos = interp_llh(gt_times, gt_llh, ep["gpst"])
        if gt_pos is None:
            continue
        sol_ecef = llh_to_ecef(ep["lat"], ep["lon"], ep["h"])
        gt_ecef = llh_to_ecef(*gt_pos)
        sol_enu = ecef_to_enu(*sol_ecef, ref_ecef, ref_lat, ref_lon)
        gt_enu = ecef_to_enu(*gt_ecef, ref_ecef, ref_lat, ref_lon)
        e = sol_enu[0] - gt_enu[0]
        n = sol_enu[1] - gt_enu[1]
        u = sol_enu[2] - gt_enu[2]
        h_vals.append(math.hypot(e, n))
        e_vals.append(e)
        n_vals.append(n)
        u_vals.append(u)
        if "yaw" in ep:
            gt_hdg = interp_scalar(gt_times, gt["heading"], ep["gpst"], angular=True)
            if gt_hdg is not None:
                yaw_err_vals.append(angle_error_deg(ep["yaw"], gt_hdg))

    if not h_vals:
        raise RuntimeError(f"No matched epochs for {solution}")

    def rmse(vals: list[float]) -> float:
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    return {
        "n_solution": len(epochs),
        "n_matched": len(h_vals),
        "rmse_h_m": rmse(h_vals),
        "rmse_e_m": rmse(e_vals),
        "rmse_n_m": rmse(n_vals),
        "rmse_u_m": rmse(u_vals),
        "yaw_rmse_deg": rmse(yaw_err_vals) if yaw_err_vals else None,
        "fixed_rate": quality_counts.get(4, 0) / len(epochs),
        "quality_counts": {str(k): v for k, v in sorted(quality_counts.items())},
    }


def validate_dataset(ds: Dataset) -> dict[str, Any]:
    paths = {
        "gt": ds.root / ds.gt_file,
        "rover": ds.root / ds.rover,
        "base": ds.root / ds.base,
        "eph": ds.root / ds.eph,
        "dcb": REPO / ds.dcb,
        "imu": ds.root / "gici_rrr/imu.bin.txt",
        "camera": ds.root / "gici_rrr/camera.bin",
        "gici_main": GICI_MAIN,
        "template": TEMPLATE,
    }
    checks = {k: {"path": str(v), "ok": v.is_file()} for k, v in paths.items()}
    return {"dataset": ds.key, "title": ds.title, "checks": checks, "ok": all(c["ok"] for c in checks.values())}


def render_config(ds: Dataset, out_dir: Path) -> Path:
    (out_dir / "output").mkdir(parents=True, exist_ok=True)
    text = TEMPLATE.read_text()
    cam_buffer = 672 * 376 + 512
    repl = {
        "<ROVER_OBS>": str(ds.root / ds.rover),
        "<REF_OBS>": str(ds.root / ds.base),
        "<EPH_NAV>": str(ds.root / ds.eph),
        "<DCB_FILE>": str(REPO / ds.dcb),
        "<IMU_FILE>": str(ds.root / "gici_rrr/imu.bin.txt"),
        "<CAMERA_FILE>": str(ds.root / "gici_rrr/camera.bin"),
        "<CAM_BUFFER>": str(cam_buffer),
        "<OUTPUT_DIR>": str(out_dir / "output"),
    }
    for k, v in repl.items():
        text = text.replace(k, v)
    cfg_path = out_dir / "config.yaml"
    cfg_path.write_text(text)
    return cfg_path


def run_gici(cfg_path: Path, out_dir: Path, timeout_s: int, skip_run: bool) -> dict[str, Any]:
    sol = out_dir / "output" / "solution.txt"
    log_path = out_dir / "run.log"
    if skip_run and sol.is_file() and sol.stat().st_size > 0:
        return {"skipped_run": True}
    if sol.exists():
        sol.unlink()
    t0 = time.time()
    cmd = ["timeout", "-s", "INT", str(timeout_s), str(GICI_MAIN), str(cfg_path)]
    with log_path.open("w") as log_fp:
        proc = subprocess.run(cmd, stdout=log_fp, stderr=subprocess.STDOUT, cwd=str(REPO), check=False)
    meta = {"exit_code": proc.returncode, "wall_s": time.time() - t0, "skipped_run": False}
    if proc.returncode not in (0, 124, 130, -6, 134) and (not sol.is_file() or sol.stat().st_size == 0):
        raise RuntimeError(f"gici_main failed exit={proc.returncode}; see {log_path}")
    if not sol.is_file() or sol.stat().st_size == 0:
        raise RuntimeError(f"no solution.txt; see {log_path}")
    return meta


def run_dataset(ds: Dataset, out_root: Path, skip_run: bool) -> dict[str, Any]:
    out_dir = out_root / ds.key
    out_dir.mkdir(parents=True, exist_ok=True)
    validation = validate_dataset(ds)
    (out_dir / "dataset_validation.json").write_text(json.dumps(validation, indent=2))
    if not validation["ok"]:
        raise RuntimeError(f"dataset validation failed for {ds.key}")

    cfg_path = render_config(ds, out_dir)
    meta = run_gici(cfg_path, out_dir, ds.timeout_s, skip_run=skip_run)
    gt = load_ground_truth(ds.root / ds.gt_file)
    metrics = evaluate_solution(out_dir / "output" / "solution.txt", gt, ds.gps_week_day_offset)
    paper = PAPER_APE[ds.key]
    result = {
        "dataset": ds.key,
        "title": ds.title,
        "paper_ape_pos_m": paper["pos_m"],
        "paper_ape_rot_deg": paper["rot_deg"],
        "metrics": metrics,
        "meta": meta,
        "solution_lines": sum(1 for _ in (out_dir / "output" / "solution.txt").open()),
    }
    (out_dir / "metrics.json").write_text(json.dumps(result, indent=2))
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description="UrbanNav upstream RRR baseline")
    ap.add_argument("datasets", nargs="*", choices=["medium", "deep"], default=["medium", "deep"])
    ap.add_argument("--skip-run", action="store_true")
    ap.add_argument("--validate-only", action="store_true")
    ap.add_argument("--out-root", type=Path, default=REPO / "results" / "baseline" / "urbannav")
    args = ap.parse_args()
    if args.datasets == ["medium", "deep"] and len(sys.argv) == 1:
        pass

    args.out_root.mkdir(parents=True, exist_ok=True)
    results: dict[str, dict] = {}
    failures: list[str] = []

    for key in args.datasets:
        ds = DATASETS[key]
        print(f"\n=== {ds.title} ===", flush=True)
        try:
            if args.validate_only:
                v = validate_dataset(ds)
                print(json.dumps(v, indent=2))
                if not v["ok"]:
                    failures.append(key)
                continue
            row = run_dataset(ds, args.out_root, skip_run=args.skip_run)
            results[key] = row
            m = row["metrics"]
            print(
                f"  rmse_h={m['rmse_h_m']:.3f} m  yaw_rmse={m['yaw_rmse_deg']:.3f}°  "
                f"matched={m['n_matched']}  fixed={100*m['fixed_rate']:.1f}%  "
                f"wall={row['meta'].get('wall_s', 0):.0f}s",
                flush=True,
            )
            print(
                f"  paper ref: {row['paper_ape_pos_m']:.2f} m / {row['paper_ape_rot_deg']:.2f}°",
                flush=True,
            )
        except Exception as exc:
            failures.append(f"{key}: {exc}")
            print(f"  FAILED: {exc}", flush=True)

    if args.validate_only:
        return 0 if not failures else 1

    summary_path = args.out_root / "summary.json"
    summary_path.write_text(json.dumps(results, indent=2))
    print(f"\nWrote {summary_path}")
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
