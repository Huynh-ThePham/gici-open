#!/usr/bin/env python3
"""Run author UrbanNav RTK/IMU/Camera RRR baseline (f2b8579 templates only).

Uses research/config/rtk_imu_camera_rrr_urbannav.yaml derived from:
  - option/post_estimation_RTK_RRR_rinex_imutext.yaml
  - ros_wrapper/.../ros_urbannav.yaml RRR block

Does NOT use UrbanNavDataset/gici_rrr/config.yaml or any paper1 fork.

Watchdog (default on): stops gici_main with SIGINT when solution.txt stops growing
for URBANNAV_FILEMODE_STABLE_SECONDS (default 90s) after min GPGGA count is met.
Disable: --no-watchdog or URBANNAV_FILEMODE_WATCH=0.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO = Path(__file__).resolve().parents[1]
GICI_MAIN = Path(
    os.environ.get(
        "GICI_MAIN",
        str(REPO / "build" / "gici_main"),
    )
)
BASELINE_TEMPLATE = REPO / "research" / "config" / "rtk_imu_camera_rrr_urbannav.yaml"
RESEARCH_VA_TEMPLATE = REPO / "research" / "config" / "rtk_imu_camera_rrr_va_urbannav.yaml"


def _default_data_root() -> Path:
    """Resolve UrbanNav parent directory that contains Medium/Deep folders.

    Honors URBANNAV_DATA_ROOT only if it actually contains the datasets; a stale
    env pointing at a missing UrbanNavDataset/ tree must not win over a valid
    layout under /media/.../dataset/.
    """
    names = ("UrbanNav-HK-Medium-Urban-1", "UrbanNav-HK-Deep-Urban-1")

    def _ok(root: Path) -> bool:
        return any((root / name).is_dir() for name in names)

    env_root = os.environ.get("URBANNAV_DATA_ROOT")
    if env_root:
        p = Path(env_root)
        if _ok(p):
            return p
    candidates = [
        # Re-prepared layout (2026-07): sequences live under an extra UrbanNav/ level.
        Path("/media/theph/Data1/Research/dataset/UrbanNav"),
        Path("/media/theph/Data1/Research/dataset"),
        Path("/media/theph/Data1/Research/dataset/UrbanNavDataset"),
    ]
    for candidate in candidates:
        if _ok(candidate):
            return candidate
    return Path(env_root) if env_root else candidates[0]


DATA_ROOT = _default_data_root()
GPS_UTC_LEAP_SECONDS = 18.0
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

PAPER_APE = {
    "medium": {"pos_m": 3.40, "rot_deg": 1.30},
    "deep": {"pos_m": 2.46, "rot_deg": 1.64},
    # Chi et al. Table V UrbanNav Harsh (Mongkok) RRR reference.
    "harsh": {"pos_m": 6.73, "rot_deg": 1.44},
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
    min_gpgga_epochs: int
    # Full expected GPGGA epoch count = the completion target (0 -> fall back to
    # min_gpgga_epochs). A plateau ABOVE min_gpgga_epochs but BELOW this is a stall-
    # induced TRUNCATION, not end-of-stream, and must not be accepted as complete.
    expected_gpgga: int = 0
    # Official-eval GT quality gate (Inertial-Explorer Q <= gt_q_max). None = no gate.
    gt_q_max: int | None = None


DATASETS: dict[str, Dataset] = {
    "medium": Dataset(
        key="medium",
        title="UrbanNav-HK-Medium-Urban-1",
        root=DATA_ROOT / "UrbanNav-HK-Medium-Urban-1",
        gt_file="UrbanNav_TST_GT_raw.txt",
        gps_week_day_offset=86400.0,
        rover="gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs",
        # HKKT 5 s session base (h02, HK SatRef archive) — standardized rate across all 3.
        base="gnss/base/hkkt137_5s.rnx",
        eph="gnss/base/brdc1370.rnx",
        dcb="research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX",
        timeout_s=7200,
        min_gpgga_epochs=6500,
        expected_gpgga=7579,
    ),
    "deep": Dataset(
        key="deep",
        title="UrbanNav-HK-Deep-Urban-1",
        root=DATA_ROOT / "UrbanNav-HK-Deep-Urban-1",
        gt_file="UrbanNav_whampoa_raw.txt",
        gps_week_day_offset=432000.0,
        rover="gnss/UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs",
        # HKKT 5 s session base (h06, HK SatRef archive) — standardized rate across all 3.
        # (Equivalent to the prior _deprecated/hkkt141g.21o 5 s cut that gave 2.14 m.)
        base="gnss/base/hkkt141_5s.rnx",
        eph="gnss/base/_deprecated/brdc_mn.rnx",
        dcb="research/dcb/CAS0MGXRAP_20211410000_01D_01D_DCB.BSX",
        timeout_s=10800,
        min_gpgga_epochs=14000,
        expected_gpgga=15119,
    ),
    "harsh": Dataset(
        key="harsh",
        title="UrbanNav-HK-Harsh-Urban-1",
        root=DATA_ROOT / "UrbanNav-HK-Harsh-Urban-1",
        gt_file="UrbanNav_mongkok_GT_part_raw.txt",
        # DOY 138 = 2021-05-18 (Tue) = GPS week 2158 day 2.
        gps_week_day_offset=172800.0,
        rover="gnss/UrbanNav-HK-Harsh-Urban-1.ublox.f9p.splitter.obs",
        # HKKT 5 s session base (h03+h04 merged, HK SatRef archive) — standardized
        # rate across all 3. eph BKG mixed nav; DCB CAS 2021-138. Same sources as Deep/Medium.
        base="gnss/base/hkkt138_5s.rnx",
        eph="gnss/base/brdc1380.rnx",
        dcb="research/dcb/CAS0MGXRAP_20211380000_01D_01D_DCB.BSX",
        timeout_s=14400,
        min_gpgga_epochs=28000,
        expected_gpgga=32609,
        # Official Harsh eval: GT is partial (t_rel [453,2764]s, auto-windowed by GT
        # coverage) and low quality; gate to Q <= 2 per the dataset audit.
        gt_q_max=2,
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
    qflag: list[int] = []
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
            # Inertial-Explorer quality flag (1 best .. 6 worst); UrbanNav GT col 20.
            q = int(float(parts[19]))
        except ValueError:
            continue
        times.append(gpst)
        llh.append((lat, lon, h))
        heading.append(hdg)
        qflag.append(q)
    if not times:
        raise RuntimeError(f"No ground truth parsed from {path}")
    return {"times": times, "llh": llh, "heading": heading, "qflag": qflag}


def gt_q_ok(times: list[float], qflag: list[int], t: float, q_max: int) -> bool:
    """True iff the two GT epochs bracketing t both satisfy Q <= q_max.

    Conservative: an evaluated solution epoch is kept only when it lies between
    two good-quality GT fixes, so interpolated reference positions are trustworthy.
    """
    if t < times[0] or t > times[-1]:
        return False
    lo, hi = 0, len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    return qflag[lo] <= q_max and qflag[hi] <= q_max


def parse_solution(path: Path, gps_week_day_offset: float) -> list[dict[str, Any]]:
    esa_by_time: dict[str, dict[str, float]] = {}
    for line in path.read_text(errors="replace").splitlines():
        if not line.startswith("$GPESA,"):
            continue
        parts = line.split("*", 1)[0].split(",")
        if len(parts) < 8:
            continue
        try:
            # GICI's internal yaw is initialized as -atan2(Ve, Vn) (see
            # gnss_imu_initializer.cpp), the negative of standard compass
            # heading atan2(Ve, Vn); negate here so it's comparable to GT heading.
            esa_by_time[parts[1]] = {
                "yaw": (-float(parts[7])) % 360.0,
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


def evaluate_solution(
    solution: Path,
    gt: dict[str, list[Any]],
    gps_week_day_offset: float,
    q_max: int | None = None,
) -> dict[str, Any]:
    epochs = parse_solution(solution, gps_week_day_offset)
    if not epochs:
        raise RuntimeError(f"No GPGGA epochs in {solution}")

    gt_times = gt["times"]
    gt_llh = gt["llh"]
    gt_q = gt.get("qflag")
    ref_lat, ref_lon, ref_h = gt_llh[0]
    ref_ecef = llh_to_ecef(ref_lat, ref_lon, ref_h)

    e_vals, n_vals, u_vals, h_vals, yaw_err_vals = [], [], [], [], []
    quality_counts: dict[int, int] = {}
    n_q_filtered = 0

    for ep in epochs:
        quality_counts[ep["quality"]] = quality_counts.get(ep["quality"], 0) + 1
        gt_pos = interp_llh(gt_times, gt_llh, ep["gpst"])
        if gt_pos is None:
            continue
        # Partial/low-quality GT (e.g. UrbanNav Harsh): keep only epochs bracketed
        # by GT fixes of quality Q <= q_max. The [453,2764]s Harsh window is already
        # enforced by GT coverage (interp_llh returns None outside it).
        if q_max is not None and gt_q is not None:
            if not gt_q_ok(gt_times, gt_q, ep["gpst"], q_max):
                n_q_filtered += 1
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
        "float_rate": quality_counts.get(5, 0) / len(epochs),
        "single_rate": quality_counts.get(1, 0) / len(epochs),
        "quality_counts": {str(k): v for k, v in sorted(quality_counts.items())},
        "gt_q_max": q_max,
        "n_gt_q_filtered": n_q_filtered,
    }


def validate_dataset(ds: Dataset, template_path: Path, config_source: str) -> dict[str, Any]:
    paths = {
        "gt": ds.root / ds.gt_file,
        "rover": ds.root / ds.rover,
        "base": ds.root / ds.base,
        "eph": ds.root / ds.eph,
        "dcb": REPO / ds.dcb,
        "imu": ds.root / "gici_rrr/imu.bin.txt",
        "camera": ds.root / "gici_rrr/camera.bin",
        "gici_main": GICI_MAIN,
        "template": template_path,
    }
    if config_source == "dataset":
        paths["dataset_config"] = ds.root / "gici_rrr/config.yaml"
    checks = {k: {"path": str(v), "ok": v.is_file()} for k, v in paths.items()}
    return {
        "dataset": ds.key,
        "title": ds.title,
        "data_root": str(DATA_ROOT),
        "config_source": config_source,
        "checks": checks,
        "ok": all(c["ok"] for c in checks.values()),
    }


def _streamer_by_tag(config: dict[str, Any], tag: str) -> dict[str, Any]:
    for item in config["stream"]["streamers"]:
        streamer = item.get("streamer", {})
        if streamer.get("tag") == tag:
            return streamer
    raise KeyError(f"streamer tag not found: {tag}")


def _render_dataset_config(ds: Dataset, out_dir: Path) -> str:
    cfg = yaml.safe_load((ds.root / "gici_rrr/config.yaml").read_text())
    _streamer_by_tag(cfg, "str_dcb_file")["path"] = str(REPO / ds.dcb)
    for tag in ("str_rrr_solution_file", "str_solution_file"):
        try:
            _streamer_by_tag(cfg, tag)["path"] = str(out_dir / "output" / "solution.txt")
            break
        except KeyError:
            continue

    if "logging" in cfg:
        cfg["logging"]["file_directory"] = str(out_dir / "log")

    estimator = cfg["estimate"][0]["estimator"]
    base_opts = estimator.get("estimator_base_options", {})
    if "log_intermediate_data_directory" in base_opts:
        base_opts["log_intermediate_data_directory"] = str(out_dir / "intermediate")

    return yaml.safe_dump(cfg, sort_keys=False)


def _assert_estimator_config(text: str, expected_estimator: str, context: str) -> None:
    cfg = yaml.safe_load(text)
    try:
        estimator = cfg["estimate"][0]["estimator"]
    except (KeyError, IndexError, TypeError) as exc:
        raise RuntimeError(f"{context}: invalid estimator config structure") from exc

    actual = estimator.get("type")
    if actual != expected_estimator:
        raise RuntimeError(
            f"{context}: expected estimator type {expected_estimator!r}, got {actual!r}"
        )

    rrr_options = estimator.get("rtk_imu_camera_rrr_options", {})
    uses_va = bool(rrr_options.get("use_vision_aided_ambiguity_resolution", False))
    if expected_estimator == "rtk_imu_camera_rrr" and uses_va:
        raise RuntimeError(
            f"{context}: baseline config must not enable vision-aided ambiguity resolution"
        )
    if expected_estimator == "rtk_imu_camera_rrr_va" and not uses_va:
        raise RuntimeError(
            f"{context}: VA config must explicitly enable vision-aided ambiguity resolution"
        )


def render_config(
    ds: Dataset,
    out_dir: Path,
    config_source: str,
    template_path: Path,
    expected_estimator: str,
) -> Path:
    (out_dir / "output").mkdir(parents=True, exist_ok=True)
    if config_source == "dataset":
        text = _render_dataset_config(ds, out_dir)
    else:
        text = template_path.read_text()
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
    _assert_estimator_config(text, expected_estimator, str(template_path))
    cfg_path = out_dir / "config.yaml"
    cfg_path.write_text(text)
    return cfg_path


def count_gpgga(path: Path) -> int:
    if not path.is_file():
        return 0
    n = 0
    with path.open(errors="replace") as fp:
        for line in fp:
            if line.startswith("$GPGGA,"):
                n += 1
    return n


def _log_has_fatal(path: Path) -> bool:
    if not path.is_file():
        return False
    text = path.read_text(errors="replace")
    return any(
        marker in text
        for marker in (
            "Check failed:",
            "segment fault",
            "Segmentation fault",
            "FATAL",
            "core dumped",
            "Aborted at",
            "terminate called",
        )
    )


def _watchdog_settings(ds: Dataset | None, use_watchdog: bool) -> dict[str, float | int | bool]:
    enabled = use_watchdog and os.environ.get("URBANNAV_FILEMODE_WATCH", "1") != "0"
    default_min = ds.min_gpgga_epochs if ds else 5000
    min_gpgga = int(os.environ.get("URBANNAV_FILEMODE_MIN_GPGGA", str(default_min)))
    default_expected = ds.expected_gpgga if (ds and ds.expected_gpgga) else default_min
    expected = int(os.environ.get("URBANNAV_FILEMODE_EXPECTED_GPGGA", str(default_expected)))
    # Clean completion requires reaching (near) the full expected epoch count -- NOT merely
    # the sanity floor. Using the floor let a transient near-end stall (plateau above the
    # floor for stable_s) be misread as end-of-stream and truncated (deep/baseline run2,
    # 2026-07-21). Small tolerance absorbs benign run-to-run epoch jitter.
    complete_target = max(min_gpgga, expected - max(5, int(0.003 * expected)))
    return {
        "enabled": enabled,
        "interval_s": float(os.environ.get("URBANNAV_FILEMODE_WATCH_INTERVAL", "10")),
        "stable_s": float(os.environ.get("URBANNAV_FILEMODE_STABLE_SECONDS", "90")),
        "min_gpgga": min_gpgga,
        "expected_gpgga": expected,
        "complete_target": complete_target,
        "progress_every_s": float(os.environ.get("URBANNAV_FILEMODE_PROGRESS_EVERY", "30")),
        # Hang guard: gici_main can freeze near end-of-stream (known race). If the solution
        # stops growing for this long, stop it so the run can't wedge until timeout. Raised
        # 150->240s so a transient near-end stall has time to RESUME to the full count
        # (avoiding truncation) before the guard fires.
        "hang_s": float(os.environ.get("URBANNAV_FILEMODE_HANG_SECONDS", "240")),
    }


def run_gici(
    cfg_path: Path,
    out_dir: Path,
    timeout_s: int,
    skip_run: bool,
    *,
    ds: Dataset | None = None,
    use_watchdog: bool = True,
) -> dict[str, Any]:
    sol = out_dir / "output" / "solution.txt"
    log_path = out_dir / "run.log"
    wd = _watchdog_settings(ds, use_watchdog)
    if skip_run and sol.is_file() and sol.stat().st_size > 0:
        gpgga = count_gpgga(sol)
        if gpgga < int(wd["min_gpgga"]):
            raise RuntimeError(
                f"--skip-run solution has only {gpgga} GPGGA epochs; "
                f"expected at least {wd['min_gpgga']}: {sol}"
            )
        return {"skipped_run": True, "final_gpgga": gpgga, "watchdog": wd}
    if sol.exists():
        sol.unlink()

    t0 = time.time()
    cmd = [str(GICI_MAIN), str(cfg_path)]
    meta: dict[str, Any] = {"skipped_run": False, "watchdog": wd}

    with log_path.open("w") as log_fp:
        proc = subprocess.Popen(
            cmd,
            stdout=log_fp,
            stderr=subprocess.STDOUT,
            cwd=str(REPO),
            start_new_session=True,
        )
        stable_elapsed = 0.0
        hang_elapsed = 0.0
        prev_gpgga = -1
        prev_size = -1
        next_progress = t0 + float(wd["progress_every_s"])

        try:
            while proc.poll() is None:
                now = time.time()
                elapsed = now - t0
                if elapsed >= timeout_s:
                    print(
                        f"  [watchdog] max timeout {timeout_s}s reached; sending SIGINT",
                        flush=True,
                    )
                    meta["timeout_reached"] = True
                    os.killpg(proc.pid, signal.SIGINT)
                    break

                time.sleep(float(wd["interval_s"]))

                if _log_has_fatal(log_path):
                    proc.terminate()
                    proc.wait(timeout=10)
                    raise RuntimeError(f"gici_main fatal error; see {log_path}")

                gpgga = count_gpgga(sol)
                size = sol.stat().st_size if sol.is_file() else 0

                if now >= next_progress:
                    print(
                        f"  [watchdog] elapsed={elapsed:.0f}s gpgga={gpgga} "
                        f"stable={stable_elapsed:.0f}/{wd['stable_s']:.0f}s",
                        flush=True,
                    )
                    next_progress = now + float(wd["progress_every_s"])

                if not wd["enabled"]:
                    prev_gpgga, prev_size = gpgga, size
                    continue

                if gpgga >= int(wd["complete_target"]) and gpgga == prev_gpgga:
                    stable_elapsed += float(wd["interval_s"])
                    if stable_elapsed >= float(wd["stable_s"]):
                        print(
                            f"  [watchdog] solution stable {stable_elapsed:.0f}s "
                            f"(gpgga={gpgga} >= target {wd['complete_target']}); sending SIGINT",
                            flush=True,
                        )
                        os.killpg(proc.pid, signal.SIGINT)
                        meta["watchdog_stopped"] = True
                        meta["watchdog_gpgga"] = gpgga
                        break
                else:
                    stable_elapsed = 0.0

                # Hang guard (independent of min_gpgga): frozen solution => stuck.
                if gpgga > 100 and gpgga == prev_gpgga:
                    hang_elapsed += float(wd["interval_s"])
                    if hang_elapsed >= float(wd["hang_s"]):
                        print(
                            f"  [watchdog] solution FROZEN {hang_elapsed:.0f}s "
                            f"(gpgga={gpgga}); gici_main likely hung, sending SIGINT",
                            flush=True,
                        )
                        os.killpg(proc.pid, signal.SIGINT)
                        meta["hang_stopped"] = True
                        meta["watchdog_gpgga"] = gpgga
                        break
                else:
                    hang_elapsed = 0.0

                prev_gpgga, prev_size = gpgga, size

            try:
                proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=10)
        except KeyboardInterrupt:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=30)

    meta["exit_code"] = proc.returncode if proc.returncode is not None else -1
    meta["wall_s"] = time.time() - t0
    if meta.get("timeout_reached"):
        raise RuntimeError(f"gici_main timed out after {timeout_s}s; see {log_path}")
    ok_exits = {0, 130, -signal.SIGINT}
    if meta["exit_code"] not in ok_exits:
        raise RuntimeError(f"gici_main failed exit={meta['exit_code']}; see {log_path}")
    if not sol.is_file() or sol.stat().st_size == 0:
        raise RuntimeError(f"no solution.txt; see {log_path}")
    final_gpgga = count_gpgga(sol)
    meta["final_gpgga"] = final_gpgga
    meta["expected_gpgga"] = int(wd["expected_gpgga"])
    target = int(wd["complete_target"])
    meta["truncated"] = final_gpgga < target
    if final_gpgga >= target:
        pass  # full-length run
    elif final_gpgga >= int(wd["min_gpgga"]):
        # Above the sanity floor but below the full expected count: a stall/hang stopped
        # the run early (the deep/baseline run2 truncation). NEVER silently accept a
        # truncated run -- remove the partial so a resume cannot skip it, and fail so the
        # caller re-runs it (mirrors run_gici_canonical.sh reason-based completion).
        try:
            sol.unlink()
        except OSError:
            pass
        raise RuntimeError(
            f"truncated run: {final_gpgga} GPGGA epochs < completion target {target} "
            f"(expected ~{wd['expected_gpgga']}); removed partial, re-run: {log_path}"
        )
    elif meta.get("hang_stopped") and final_gpgga >= 0.85 * int(wd["min_gpgga"]):
        # Genuine hang below even the sanity floor but still >=85% of it: accept as a
        # flagged partial (e.g. Harsh, whose ground truth is itself partial).
        meta["short_run"] = True
        print(
            f"  [watchdog] accepting partial (hang): {final_gpgga} GPGGA "
            f">= 85% of {wd['min_gpgga']}",
            flush=True,
        )
    else:
        raise RuntimeError(
            f"solution has only {final_gpgga} GPGGA epochs; "
            f"expected at least {wd['min_gpgga']}: {sol}"
        )
    return meta


def run_dataset(
    ds: Dataset,
    out_root: Path,
    skip_run: bool,
    config_source: str,
    template_path: Path,
    expected_estimator: str,
    result_algorithm: str,
    use_watchdog: bool = True,
) -> dict[str, Any]:
    out_dir = out_root / ds.key
    out_dir.mkdir(parents=True, exist_ok=True)
    validation = validate_dataset(ds, template_path, config_source)
    (out_dir / "dataset_validation.json").write_text(json.dumps(validation, indent=2))
    if not validation["ok"]:
        raise RuntimeError(f"dataset validation failed for {ds.key}")

    cfg_path = render_config(ds, out_dir, config_source, template_path, expected_estimator)
    meta = run_gici(cfg_path, out_dir, ds.timeout_s, skip_run=skip_run, ds=ds, use_watchdog=use_watchdog)
    gt = load_ground_truth(ds.root / ds.gt_file)
    metrics = evaluate_solution(
        out_dir / "output" / "solution.txt", gt, ds.gps_week_day_offset, q_max=ds.gt_q_max
    )
    paper = PAPER_APE[ds.key]
    result = {
        "dataset": ds.key,
        "title": ds.title,
        "algorithm": result_algorithm,
        "paper_ape_pos_m": paper["pos_m"],
        "paper_ape_rot_deg": paper["rot_deg"],
        "config_source": config_source,
        "config_template": str(template_path),
        "metrics": metrics,
        "meta": meta,
        "solution_lines": sum(1 for _ in (out_dir / "output" / "solution.txt").open()),
    }
    (out_dir / "metrics.json").write_text(json.dumps(result, indent=2))
    return result


def main(
    argv: list[str] | None = None,
    *,
    description: str = "UrbanNav upstream RRR baseline",
    template_path: Path = BASELINE_TEMPLATE,
    expected_estimator: str = "rtk_imu_camera_rrr",
    result_algorithm: str = "rtk_imu_camera_rrr",
    default_out_root: Path | None = None,
) -> int:
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("datasets", nargs="*", choices=["medium", "deep", "harsh"], default=["medium", "deep"])
    ap.add_argument("--skip-run", action="store_true")
    ap.add_argument(
        "--no-watchdog",
        action="store_true",
        help="Disable solution-stable watchdog (legacy timeout-only behaviour)",
    )
    ap.add_argument("--validate-only", action="store_true")
    ap.add_argument(
        "--config-source",
        choices=["dataset", "wrapper"],
        default="wrapper",
        help="wrapper = research/config/rtk_imu_camera_rrr_urbannav.yaml (canonical); "
        "dataset = UrbanNav gici_rrr/config.yaml (deprecated, do not use for new work)",
    )
    ap.add_argument(
        "--allow-dataset-config",
        action="store_true",
        help="Explicitly allow the deprecated UrbanNav dataset config source.",
    )
    ap.add_argument(
        "--out-root",
        type=Path,
        default=default_out_root or REPO / "results" / "baseline" / "urbannav",
    )
    ap.add_argument(
        "--timeout-s",
        type=int,
        default=None,
        help="Override dataset timeout_s (e.g. 21600 for Ceres-oracle Deep).",
    )
    args = ap.parse_args(argv)
    if args.config_source == "dataset" and not args.allow_dataset_config:
        ap.error("--config-source dataset is deprecated; pass --allow-dataset-config explicitly")
    if args.datasets == ["medium", "deep"] and len(sys.argv) == 1:
        pass

    args.out_root.mkdir(parents=True, exist_ok=True)
    results: dict[str, dict] = {}
    failures: list[str] = []

    for key in args.datasets:
        ds = DATASETS[key]
        if args.timeout_s is not None:
            ds = Dataset(**{**ds.__dict__, "timeout_s": int(args.timeout_s)})
        print(f"\n=== {ds.title} ===", flush=True)
        try:
            if args.validate_only:
                v = validate_dataset(ds, template_path, args.config_source)
                print(json.dumps(v, indent=2))
                if not v["ok"]:
                    failures.append(key)
                continue
            row = run_dataset(
                ds,
                args.out_root,
                skip_run=args.skip_run,
                config_source=args.config_source,
                template_path=template_path,
                expected_estimator=expected_estimator,
                result_algorithm=result_algorithm,
                use_watchdog=not args.no_watchdog,
            )
            results[key] = row
            m = row["metrics"]
            print(
                f"  rmse_h={m['rmse_h_m']:.3f} m  yaw_rmse={m['yaw_rmse_deg']:.3f}°  "
                f"matched={m['n_matched']}  fixed={100*m['fixed_rate']:.1f}%  "
                f"wall={row['meta'].get('wall_s', 0):.0f}s"
                + (
                    "  watchdog=1"
                    if row["meta"].get("watchdog_stopped")
                    else ""
                ),
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
