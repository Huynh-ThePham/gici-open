#!/usr/bin/env python3
"""Lock UrbanNav Medium through audit phases P2–P10.

P2  Audit GNSS
P3  Audit timestamp alignment
P4  Audit IMU
P5  Audit camera
P6  Audit extrinsic vs config
P7  Run RTK-only
P8  Run RTK–IMU TC
P9  Run full RRR
P10 Audit output vs ground truth (author APE; PASS only if metrics match locked reference)

Usage:
  python3 scripts/lock_urbannav_medium.py [--root PATH] [--skip-run]
"""

from __future__ import annotations

import argparse
import calendar
import hashlib
import json
import math
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

REPO = Path(__file__).resolve().parents[1]
GICI_MAIN = Path(os.environ.get("GICI_MAIN", str(REPO / "build" / "gici_main")))
DEFAULT_ROOT = Path(
    os.environ.get(
        "URBANNAV_DATA_ROOT",
        "/media/theph/Data1/Research/dataset/UrbanNavDataset",
    )
)
SCENE = "UrbanNav-HK-Medium-Urban-1"
GPS_UTC_LEAP = 18.0
GPS_WEEK_DAY_OFFSET = 86400.0

TPL_RTK = REPO / "research/config/rtk_gnss_urbannav_filemode.yaml"
TPL_RTK_IMU = REPO / "research/config/rtk_imu_tc_urbannav_filemode.yaml"
TPL_RRR = REPO / "research/config/rtk_imu_camera_rrr_urbannav.yaml"
DCB = REPO / "research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX"
EXPECTED = REPO / "research/baseline/expected_urbannav_medium.json"

ACCEPTED_EXIT = {0, -signal.SIGINT, 130}


def md5(path: Path) -> str:
    h = hashlib.md5()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_rinex_epochs(path: Path, max_epochs: int = 50000) -> list[float]:
    """Return GPS week-second style epoch markers from RINEX obs (approx UTC via leap)."""
    epochs: list[float] = []
    epoch_re = re.compile(
        r"^>\s*(\d{4})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+(\d{1,2})\s+([\d.]+)"
    )
    with path.open(errors="replace") as fp:
        for line in fp:
            m = epoch_re.match(line)
            if not m:
                continue
            y, mo, d, h, mi, sec = map(float, m.groups())
            # UrbanNav medium 2021-05-17; use ordinal approximation for span audit only
            utc = calendar.timegm((int(y), int(mo), int(d), int(h), int(mi), int(sec)))
            # RINEX obs epochs are GPS time; GICI/IMU/camera use UTC Unix.
            epochs.append(float(utc - GPS_UTC_LEAP))
            if len(epochs) >= max_epochs:
                break
    return epochs


def read_imu_times(path: Path) -> list[float]:
    times: list[float] = []
    with path.open(errors="replace") as fp:
        next(fp, None)
        for line in fp:
            parts = line.split()
            if not parts:
                continue
            try:
                times.append(float(parts[0]))
            except ValueError:
                continue
    return times


def read_camera_times(path: Path) -> list[float]:
    times: list[float] = []
    with path.open() as fp:
        next(fp, None)
        for line in fp:
            parts = line.strip().split(",")
            if len(parts) >= 2:
                times.append(float(parts[1]))
    return times


def overlap_stats(a: list[float], b: list[float]) -> dict[str, Any]:
    if not a or not b:
        return {"ok": False, "reason": "empty series"}
    start = max(min(a), min(b))
    end = min(max(a), max(b))
    return {
        "ok": end > start,
        "overlap_s": max(0.0, end - start),
        "a_span_s": max(a) - min(a),
        "b_span_s": max(b) - min(b),
        "a_start": min(a),
        "a_end": max(a),
        "b_start": min(b),
        "b_end": max(b),
        "common_start": start,
        "common_end": end,
    }


def render_config(template: Path, out: Path, mapping: dict[str, str]) -> Path:
    text = template.read_text()
    for k, v in mapping.items():
        text = text.replace(k, v)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    return out


def count_gpgga(path: Path) -> int:
    if not path.is_file():
        return 0
    n = 0
    with path.open(errors="replace") as fp:
        for line in fp:
            if line.startswith("$GPGGA,"):
                n += 1
    return n


def run_gici(
    cfg: Path,
    out_dir: Path,
    log_name: str,
    timeout_s: int,
    *,
    min_gpgga: int = 400,
    stable_s: float = 90.0,
) -> dict[str, Any]:
    sol = out_dir / "solution.txt"
    log = out_dir / log_name
    if sol.exists():
        sol.unlink()
    t0 = time.time()
    proc = subprocess.Popen(
        [str(GICI_MAIN), str(cfg)],
        cwd=str(REPO),
        stdout=log.open("w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    stable_elapsed = 0.0
    stall_elapsed = 0.0
    prev_gpgga = -1
    exit_code = None
    try:
        while proc.poll() is None:
            elapsed = time.time() - t0
            if elapsed >= timeout_s:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=30)
                exit_code = proc.returncode
                break
            time.sleep(10.0)
            gpgga = count_gpgga(sol)
            if gpgga > prev_gpgga:
                stall_elapsed = 0.0
            else:
                stall_elapsed += 10.0
            if gpgga >= min_gpgga and gpgga == prev_gpgga:
                stable_elapsed += 10.0
                if stable_elapsed >= stable_s:
                    os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=30)
                    exit_code = proc.returncode
                    break
            else:
                stable_elapsed = 0.0
            if stall_elapsed >= 180.0 and gpgga < min_gpgga:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=30)
                exit_code = proc.returncode
                break
            prev_gpgga = gpgga
        if exit_code is None:
            proc.wait(timeout=30)
            exit_code = proc.returncode
    except subprocess.TimeoutExpired:
        proc.kill()
        exit_code = proc.returncode

    elapsed = time.time() - t0
    gpgga = count_gpgga(sol)
    return {
        "exit_code": exit_code,
        "exit_ok": exit_code in ACCEPTED_EXIT if exit_code is not None else False,
        "elapsed_s": round(elapsed, 1),
        "solution": str(sol),
        "gpgga_epochs": gpgga,
        "solution_bytes": sol.stat().st_size if sol.is_file() else 0,
        "log": str(log),
        "watchdog": {"min_gpgga": min_gpgga, "stable_s": stable_s},
    }


def parse_matrix_from_yaml_block(text: str) -> list[float]:
    m = re.search(r"data:\s*\[(.*?)\]", text, re.S)
    if not m:
        return []
    nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", m.group(1))
    return [float(x) for x in nums]


def mat4_inv(flat: list[float]) -> list[list[float]]:
    M = [flat[i : i + 4] for i in range(0, 16, 4)]
    R = [M[r][:3] for r in range(3)]
    t = [M[r][3] for r in range(3)]
    Rt = [[R[c][r] for c in range(3)] for r in range(3)]
    tin = [-sum(Rt[r][c] * t[c] for c in range(3)) for r in range(3)]
    out = [[0.0] * 4 for _ in range(4)]
    for r in range(3):
        for c in range(3):
            out[r][c] = Rt[r][c]
        out[r][3] = tin[r]
    out[3][3] = 1.0
    return out


def audit_extrinsic(calib_dir: Path, rrr_cfg: Path) -> dict[str, Any]:
    extr = (calib_dir / "extrinsic.yaml").read_text(errors="replace")
    cfg = yaml.safe_load(rrr_cfg.read_text())
    cam_block = cfg["estimate"][0]["estimator"]["feature_handler_options"]["camera_parameters"]["cameras"][0]
    tbc_flat = cam_block["T_B_C"]["data"]
    upstream_tbc = [
        0.9988523440263594, 0.0013591158885982, 0.0478763786960621, -0.0849942494565455,
        -0.0478641883492691, -0.0079091258538426, 0.9988225393942077, 0.126,
        0.001736175887714, -0.9999677987476544, -0.0078349959194297, 0.076,
        0.0, 0.0, 0.0, 1.0,
    ]
    left = parse_matrix_from_yaml_block(extr.split("LEFT_CAMERA_T_IMU")[1][:800])
    zed_text = (calib_dir / "zed2_intrinsics.yaml").read_text(errors="replace")
    left_block = zed_text.split("RIGHT_ZED2_CAMERA")[0]

    def _f(name: str) -> float:
        m = re.search(rf"{name}:\s*([-+0-9.]+)", left_block)
        if not m:
            raise ValueError(name)
        return float(m.group(1))

    intr_zed = [_f("fx"), _f("fy"), _f("cx"), _f("cy")]
    intr_cfg = cam_block["camera"]["intrinsics"]["data"]
    intr_err = max(abs(a - b) for a, b in zip(intr_cfg, intr_zed))
    cfg_upstream_err = max(abs(a - b) for a, b in zip(tbc_flat, upstream_tbc))

    dataset_note = None
    if len(left) == 16:
        T_inv = mat4_inv(left)
        dataset_note = {
            "inv_LEFT_CAMERA_T_IMU_translation": [T_inv[r][3] for r in range(3)],
            "translation_l2_vs_config": math.sqrt(
                sum((T_inv[r][3] - tbc_flat[r * 4 + 3]) ** 2 for r in range(3))
            ),
            "note": "Dataset extrinsic.yaml uses full lever arm; GICI config uses ros_urbannav @ f2b8579 simplified T_B_C.",
        }

    ok = intr_err < 1.0 and cfg_upstream_err < 1e-6
    return {
        "ok": ok,
        "intrinsic_max_abs_diff": intr_err,
        "config_vs_upstream_ros_urbannav_max_abs_diff": cfg_upstream_err,
        "config_T_B_C_translation": [tbc_flat[r * 4 + 3] for r in range(3)],
        "dataset_extrinsic": dataset_note,
    }


def audit_p10(root: Path, sol: Path, lock_dir: Path) -> dict[str, Any]:
    gt_raw = root / "original" / "UrbanNav_TST_GT_raw.txt"
    eval_dir = lock_dir / "P10_eval"
    eval_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["GICI_BASELINE_OUT"] = str(lock_dir / "P10_baseline")
    env["URBANNAV_DATA_ROOT"] = str(root.parent)
    out_base = Path(env["GICI_BASELINE_OUT"])
    (out_base / "output").mkdir(parents=True, exist_ok=True)
    shutil_copy = sol
    dest = out_base / "output" / "solution.txt"
    if sol.resolve() != dest.resolve():
        dest.write_bytes(sol.read_bytes())

    proc = subprocess.run(
        [str(REPO / "scripts/run_author_eval_urbannav.sh"), "medium"],
        cwd=str(REPO),
        env=env,
        capture_output=True,
        text=True,
    )
    ape_json = out_base / "evaluation" / "ape_metrics.json"
    eval_ran = proc.returncode == 0 and ape_json.is_file()
    result: dict[str, Any] = {
        "eval_ran": eval_ran,
        "eval_exit": proc.returncode,
        "solution": str(sol),
        "gt": str(gt_raw),
        "ok": False,
    }
    if ape_json.is_file():
        metrics = json.loads(ape_json.read_text())
        result["ape"] = metrics
        result["paper_pass"] = metrics.get("paper_pass", False)
        result["pass_vs_locked"] = metrics.get("pass", False)
        result["ok"] = eval_ran and result["pass_vs_locked"]
        if EXPECTED.is_file():
            exp = json.loads(EXPECTED.read_text())
            locked = metrics.get("locked_reference") or exp.get("locked_reproduce_2026_07_15")
            if locked:
                result["locked_target"] = {
                    "ape_translation_rmse_m": locked["ape_translation_rmse_m"],
                    "ape_rotation_rmse_deg": locked["ape_rotation_rmse_deg"],
                }
                tol = exp.get("tolerance", {})
                result["locked_tolerance"] = tol
    else:
        result["stderr"] = proc.stderr[-4000:]
        result["ok"] = False
    return result


def run_p9_baseline_wrapper(data_root: Path, run_out: Path) -> dict[str, Any]:
    """Run full RRR via scripts/run_urbannav_rrr_baseline.py (canonical wrapper)."""
    wrapper_root = run_out / "P9_baseline_wrapper"
    wrapper_root.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["URBANNAV_DATA_ROOT"] = str(data_root)
    wrapper_log = run_out / "P9_baseline_wrapper.log"
    proc = subprocess.run(
        [
            sys.executable,
            str(REPO / "scripts/run_urbannav_rrr_baseline.py"),
            "medium",
            "--config-source",
            "wrapper",
            "--out-root",
            str(wrapper_root),
        ],
        cwd=str(REPO),
        env=env,
        capture_output=True,
        text=True,
    )
    wrapper_log.write_text((proc.stdout or "") + (proc.stderr or ""))

    medium_dir = wrapper_root / "medium"
    sol_src = medium_dir / "output" / "solution.txt"
    p9_dir = run_out / "P9_rrr"
    p9_dir.mkdir(exist_ok=True)
    sol_dst = p9_dir / "solution.txt"
    if sol_src.is_file():
        sol_dst.write_bytes(sol_src.read_bytes())

    meta: dict[str, Any] = {}
    metrics_path = medium_dir / "metrics.json"
    if metrics_path.is_file():
        meta = json.loads(metrics_path.read_text())

    gpgga = count_gpgga(sol_dst)
    exit_code = meta.get("meta", {}).get("exit_code")
    elapsed = meta.get("meta", {}).get("wall_s")
    p9: dict[str, Any] = {
        "mode": "baseline_wrapper",
        "wrapper_out": str(wrapper_root),
        "config": str(medium_dir / "config.yaml"),
        "wrapper_exit": proc.returncode,
        "exit_code": exit_code,
        "exit_ok": exit_code in ACCEPTED_EXIT if exit_code is not None else False,
        "elapsed_s": round(elapsed, 1) if elapsed is not None else None,
        "solution": str(sol_dst),
        "gpgga_epochs": gpgga,
        "solution_bytes": sol_dst.stat().st_size if sol_dst.is_file() else 0,
        "log": str(medium_dir / "run.log"),
        "wrapper_log": str(wrapper_log),
    }
    if meta:
        p9["baseline_metrics"] = meta.get("metrics")
    p9["ok"] = (
        proc.returncode == 0
        and gpgga >= 6500
        and exit_code in ACCEPTED_EXIT
    )
    if proc.returncode != 0:
        p9["stderr_tail"] = (proc.stderr or "")[-2000:]
    return p9


def write_phase(lock_dir: Path, name: str, payload: dict[str, Any]) -> None:
    path = lock_dir / f"{name}.json"
    path.write_text(json.dumps(payload, indent=2) + "\n")
    status = "PASS" if payload.get("ok", True) else "FAIL"
    print(f"[{name}] {status}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--skip-run", action="store_true", help="audits only, skip P7-P9")
    parser.add_argument("--from-phase", default="P2", help="start phase e.g. P7")
    args = parser.parse_args()

    root = args.root / SCENE
    gici = root / "gici_rrr"
    lock_dir = root / "reports" / "lock_medium"
    lock_dir.mkdir(parents=True, exist_ok=True)

    if not GICI_MAIN.is_file():
        print(f"ERROR: {GICI_MAIN} missing", file=sys.stderr)
        return 1

    phases = ["P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10"]
    start = phases.index(args.from_phase) if args.from_phase in phases else 0
    phases = phases[start:]

    common = {
        "<ROVER_OBS>": str(gici / "gnss_rover.obs"),
        "<REF_OBS>": str(gici / "gnss_reference.obs"),
        "<EPH_NAV>": str(gici / "gnss_ephemeris.nav"),
        "<DCB_FILE>": str(DCB),
        "<IMU_FILE>": str(gici / "imu.txt"),
        "<CAMERA_FILE>": str(gici / "camera.bin"),
        "<CAM_BUFFER>": str(672 * 376 + 512),
    }

    summary: dict[str, Any] = {
        "schema": "urbannav-medium-lock-v1",
        "scene": SCENE,
        "root": str(root.resolve()),
        "started_at": datetime.now(timezone.utc).isoformat(),
        "phases": {},
    }

    # P2 GNSS
    if "P2" in phases:
        rover = gici / "gnss_rover.obs"
        ref = gici / "gnss_reference.obs"
        eph = gici / "gnss_ephemeris.nav"
        rov_epochs = parse_rinex_epochs(rover)
        ref_epochs = parse_rinex_epochs(ref)
        p2 = {
            "ok": rover.is_file() and ref.is_file() and eph.is_file() and len(rov_epochs) > 100,
            "files": {
                "rover": {"path": str(rover), "md5": md5(rover), "epochs": len(rov_epochs)},
                "reference": {"path": str(ref), "md5": md5(ref), "epochs": len(ref_epochs)},
                "ephemeris": {"path": str(eph), "md5": md5(eph)},
                "dcb": {"path": str(DCB), "md5": md5(DCB)},
            },
            "rover_span_s": (max(rov_epochs) - min(rov_epochs)) if rov_epochs else 0,
            "ref_span_s": (max(ref_epochs) - min(ref_epochs)) if ref_epochs else 0,
            "rover_ref_overlap": overlap_stats(rov_epochs, ref_epochs),
        }
        write_phase(lock_dir, "P2_gnss_audit", p2)
        summary["phases"]["P2"] = p2["ok"]

    # P3 timestamps
    if "P3" in phases:
        imu_t = read_imu_times(gici / "imu.txt")
        cam_t = read_camera_times(gici / "image_timestamps.csv")
        gnss_t = parse_rinex_epochs(gici / "gnss_rover.obs")
        p3 = {
            "ok": bool(imu_t and cam_t and gnss_t),
            "counts": {"imu": len(imu_t), "camera": len(cam_t), "gnss_epochs": len(gnss_t)},
            "imu_camera_overlap": overlap_stats(imu_t, cam_t),
            "gnss_imu_overlap": overlap_stats(gnss_t, imu_t),
            "gnss_camera_overlap": overlap_stats(gnss_t, cam_t),
            "imu_rate_hz_median": (
                round(len(imu_t) / (imu_t[-1] - imu_t[0]), 2) if len(imu_t) > 1 else 0
            ),
            "camera_rate_hz_median": (
                round(len(cam_t) / (cam_t[-1] - cam_t[0]), 2) if len(cam_t) > 1 else 0
            ),
        }
        if p3["ok"]:
            p3["ok"] = (
                p3["imu_camera_overlap"]["overlap_s"] > 600
                and p3["gnss_imu_overlap"]["overlap_s"] > 600
            )
        write_phase(lock_dir, "P3_timestamp_audit", p3)
        summary["phases"]["P3"] = p3["ok"]

    # P4 IMU
    if "P4" in phases:
        imu_t = read_imu_times(gici / "imu.txt")
        dts = [imu_t[i + 1] - imu_t[i] for i in range(len(imu_t) - 1) if imu_t[i + 1] > imu_t[i]]
        p4 = {
            "ok": len(imu_t) > 100000,
            "samples": len(imu_t),
            "dt_median_s": statistics.median(dts) if dts else None,
            "dt_max_s": max(dts) if dts else None,
            "duplicate_timestamps": len(imu_t) - len(set(imu_t)),
            "span_s": imu_t[-1] - imu_t[0] if imu_t else 0,
            "rate_hz": round(len(imu_t) / (imu_t[-1] - imu_t[0]), 2) if len(imu_t) > 1 else 0,
        }
        if p4["dt_median_s"]:
            p4["ok"] = p4["ok"] and 200 <= p4["rate_hz"] <= 500 and p4["duplicate_timestamps"] == 0
        write_phase(lock_dir, "P4_imu_audit", p4)
        summary["phases"]["P4"] = p4["ok"]

    # P5 camera
    if "P5" in phases:
        cam_t = read_camera_times(gici / "image_timestamps.csv")
        cam_bin = gici / "camera.bin"
        expected_frames = len(cam_t)
        frame_bytes = 9 + (13 + 672 * 376 + 6)
        expected_size = expected_frames * frame_bytes
        size = cam_bin.stat().st_size
        mono = all(cam_t[i] <= cam_t[i + 1] for i in range(len(cam_t) - 1))
        p5 = {
            "ok": expected_frames >= 11000 and mono,
            "frames": expected_frames,
            "camera_bin_bytes": size,
            "expected_bytes_approx": expected_size,
            "size_ratio": round(size / expected_size, 4) if expected_size else 0,
            "monotonic_timestamps": mono,
            "span_s": cam_t[-1] - cam_t[0] if cam_t else 0,
            "rate_hz": round(len(cam_t) / (cam_t[-1] - cam_t[0]), 2) if len(cam_t) > 1 else 0,
        }
        p5["ok"] = p5["ok"] and 0.98 <= p5["size_ratio"] <= 1.02
        write_phase(lock_dir, "P5_camera_audit", p5)
        summary["phases"]["P5"] = p5["ok"]

    # P6 extrinsic
    if "P6" in phases:
        p6 = audit_extrinsic(root / "calibration", gici / "resolved_config.yaml")
        write_phase(lock_dir, "P6_extrinsic_audit", p6)
        summary["phases"]["P6"] = p6["ok"]

    if not args.skip_run:
        run_out = lock_dir / "runs"
        run_out.mkdir(exist_ok=True)

        if "P7" in phases:
            cfg = render_config(
                TPL_RTK,
                run_out / "P7_rtk_config.yaml",
                {**common, "<OUTPUT_DIR>": str(run_out / "P7_rtk")},
            )
            (run_out / "P7_rtk").mkdir(exist_ok=True)
            p7 = run_gici(cfg, run_out / "P7_rtk", "run.log", timeout_s=7200, min_gpgga=400, stable_s=60)
            p7["ok"] = p7["exit_ok"] and p7["gpgga_epochs"] >= 400
            write_phase(lock_dir, "P7_rtk_only", p7)
            summary["phases"]["P7"] = p7["ok"]

        if "P8" in phases:
            cfg = render_config(
                TPL_RTK_IMU,
                run_out / "P8_rtk_imu_config.yaml",
                {**common, "<OUTPUT_DIR>": str(run_out / "P8_rtk_imu")},
            )
            (run_out / "P8_rtk_imu").mkdir(exist_ok=True)
            p8 = run_gici(cfg, run_out / "P8_rtk_imu", "run.log", timeout_s=10800, min_gpgga=400, stable_s=90)
            p8["ok"] = p8["exit_ok"] and p8["gpgga_epochs"] >= 400
            write_phase(lock_dir, "P8_rtk_imu", p8)
            summary["phases"]["P8"] = p8["ok"]

        if "P9" in phases:
            p9 = run_p9_baseline_wrapper(args.root, run_out)
            write_phase(lock_dir, "P9_full_rrr", p9)
            summary["phases"]["P9"] = p9["ok"]

    if "P10" in phases:
        sol = lock_dir / "runs" / "P9_rrr" / "solution.txt"
        if not sol.is_file():
            sol = lock_dir / "runs" / "P9_baseline_wrapper" / "medium" / "output" / "solution.txt"
        if not sol.is_file():
            sol = root / "reports" / "output" / "solution.txt"
        p10 = audit_p10(root, sol, lock_dir)
        write_phase(lock_dir, "P10_output_gt_audit", p10)
        summary["phases"]["P10"] = p10["ok"]

    summary["finished_at"] = datetime.now(timezone.utc).isoformat()
    summary["all_ok"] = all(summary["phases"].values()) if summary["phases"] else False
    (lock_dir / "LOCK_SUMMARY.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["all_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
