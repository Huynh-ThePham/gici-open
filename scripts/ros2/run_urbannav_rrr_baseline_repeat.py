#!/usr/bin/env python3
"""Repeat the locked ROS 2 UrbanNav RRR baseline to measure natural variation.

Each run writes:
  output/ros2_urbannav_rrr_baseline_repeat/<dataset>/run_<nn>/
    solution.txt, node.log, ros_urbannav_rrr_ros2_adapted.yaml, output/solution.txt,
    evaluation/, run_manifest.json

Usage:
  python3 scripts/ros2/run_urbannav_rrr_baseline_repeat.py --runs 3
  python3 scripts/ros2/run_urbannav_rrr_baseline_repeat.py --runs 3 --rate 1 --skip-existing
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

REPO = Path(__file__).resolve().parents[2]
WS = REPO / "ros2_wrapper"
TEMPLATE = WS / "src/gici_ros2/config/ros_urbannav_rrr_ros2_adapted.yaml"
BASELINE_DOC = REPO / "docs/baseline/ROS2_URBANNAV_RRR_BASELINE_V1.md"
EXPECTED = REPO / "research/baseline/expected_urbannav_medium.json"
NODE_EXE = WS / "install/gici_ros2/lib/gici_ros2/gici_ros2_main"
EVAL_SCRIPT = REPO / "scripts/run_author_eval_urbannav.sh"
REPEAT_ROOT = REPO / "output/ros2_urbannav_rrr_baseline_repeat"
BASELINE_TAG = "baseline-full-rrr-ros2-v1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_paths(paths: list[Path]) -> str:
    h = hashlib.sha256()
    for path in sorted(paths):
        h.update(path.name.encode())
        h.update(path.read_bytes())
    return h.hexdigest()


def git_sha(repo: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True,
        )
        return out.stdout.strip()
    except subprocess.CalledProcessError:
        return "unknown"


def git_is_clean(repo: Path) -> bool | None:
    try:
        out = subprocess.run(
            ["git", "-C", str(repo), "status", "--porcelain"],
            check=True, capture_output=True, text=True,
        )
        return out.stdout.strip() == ""
    except subprocess.CalledProcessError:
        return None


def run_cmd(cmd: list[str], *, cwd: Path | None = None) -> tuple[int, str]:
    proc = subprocess.run(
        cmd, cwd=str(cwd or REPO), capture_output=True, text=True, check=False,
    )
    text = (proc.stdout or "") + (proc.stderr or "")
    return proc.returncode, text.strip()


def pkg_config_version(pkg: str) -> str | None:
    code, out = run_cmd(["pkg-config", "--modversion", pkg])
    return out if code == 0 else None


def cmake_package_version(pkg: str) -> str | None:
    tmp = REPO / ".tmp_cmake_probe"
    tmp.mkdir(exist_ok=True)
    cmake_file = tmp / "CMakeLists.txt"
    build_dir = tmp / "build"
    cmake_file.write_text(
        "cmake_minimum_required(VERSION 3.10)\n"
        f"project(probe_{pkg} LANGUAGES CXX)\n"
        f"find_package({pkg} REQUIRED)\n"
        f"if(DEFINED {pkg}_VERSION)\n"
        f"  message(STATUS \"VERSION=${{{pkg}_VERSION}}\")\n"
        "endif()\n"
    )
    code, out = run_cmd(["cmake", "-S", str(tmp), "-B", str(build_dir)])
    shutil.rmtree(tmp, ignore_errors=True)
    if code != 0:
        return None
    m = re.search(r"VERSION=([^\s]+)", out)
    return m.group(1) if m else "present"


def collect_toolchain() -> dict[str, Any]:
    _, gpp = run_cmd(["g++", "--version"])
    _, cmake = run_cmd(["cmake", "--version"])
    ros_distro = os.environ.get("ROS_DISTRO")
    if not ros_distro and Path("/opt/ros/humble/setup.bash").is_file():
        ros_distro = "humble"
    _, ros_version = run_cmd(["bash", "-lc", "source /opt/ros/humble/setup.bash && ros2 --version"])
    return {
        "ros_distribution": ros_distro,
        "ros2_cli_version": ros_version.splitlines()[0] if ros_version else None,
        "compiler": gpp.splitlines()[0] if gpp else None,
        "cmake": cmake.splitlines()[0] if cmake else None,
        "ceres": cmake_package_version("Ceres"),
        "opencv": pkg_config_version("opencv4"),
        "eigen3": pkg_config_version("eigen3"),
        "glog": pkg_config_version("libglog"),
    }


def collect_hardware() -> dict[str, Any]:
    model = platform.processor() or "unknown"
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                model = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    return {
        "cpu_model": model,
        "platform": platform.platform(),
        "logical_cpu_count": os.cpu_count(),
        "thread_env": {
            "OMP_NUM_THREADS": os.environ.get("OMP_NUM_THREADS"),
            "OPENBLAS_NUM_THREADS": os.environ.get("OPENBLAS_NUM_THREADS"),
            "MKL_NUM_THREADS": os.environ.get("MKL_NUM_THREADS"),
        },
    }


def dataset_paths(dataset: str) -> dict[str, Path]:
    data_root = Path(os.environ.get(
        "URBANNAV_DATA_ROOT", "/home/theph/Downloads/UrbanNavDataset-master"))
    if dataset != "medium":
        raise SystemExit("Only medium is wired for ROS 2 RRR baseline repeat.")
    gnss_dir = Path(os.environ.get(
        "URBANNAV_MEDIUM_GNSS_DIR",
        data_root / "OneDrive_1_7-11-2026/urbannav/medium"))
    sensors = Path(os.environ.get(
        "URBANNAV_MEDIUM_SENSORS",
        data_root / "UrbanNav-HK-Medium-Urban-1/ros/UrbanNav-HK_TST-20210517_sensors.bag"))
    gt = data_root / "UrbanNav-HK-Medium-Urban-1/UrbanNav_TST_GT_raw.txt"
    return {"gnss_dir": gnss_dir, "sensors": sensors, "ground_truth": gt}


def hash_dataset(dataset: str) -> tuple[str, list[str]]:
    paths = dataset_paths(dataset)
    files: list[Path] = []
    gnss_dir = paths["gnss_dir"]
    if gnss_dir.is_dir():
        files.extend(sorted(gnss_dir.rglob("*")))
    for key in ("sensors", "ground_truth"):
        p = paths[key]
        if p.is_file():
            files.append(p)
    files = [p for p in files if p.is_file()]
    if not files:
        raise SystemExit(f"No dataset files found under {paths}")
    return sha256_paths(files), [str(p) for p in files]


def ensure_bag(dataset: str) -> Path:
    bag_root = REPO / "output/ros2_urbannav_rrr/medium"
    bag_out = bag_root / "rrr_ros2"
    if bag_out.is_dir():
        return bag_out
    paths = dataset_paths(dataset)
    bag_root.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        sys.executable,
        str(REPO / "scripts/ros2/urbannav_rrr_to_ros2.py"),
        "--gnss-dir", str(paths["gnss_dir"]),
        "--sensors", str(paths["sensors"]),
        "--out", str(bag_out),
    ], check=True)
    return bag_out


def baseline_sha(config_sha256: str, binary_sha256: str, upstream_commit: str) -> str:
    payload = "|".join([
        BASELINE_TAG,
        upstream_commit,
        git_sha(REPO),
        config_sha256,
        binary_sha256,
    ])
    return hashlib.sha256(payload.encode()).hexdigest()


def kill_stale() -> None:
    subprocess.run(["pkill", "-9", "-f", "bag play .*rrr_ros2"], check=False)
    subprocess.run(["pkill", "-9", "-f", "gici_ros2_main"], check=False)
    time.sleep(1)


def parse_solution_fix_ratio(solution: Path) -> dict[str, Any]:
    if not solution.is_file():
        return {"error": "missing solution.txt"}
    quality_counts: dict[str, int] = {}
    gpgga = 0
    for line in solution.read_text(errors="replace").splitlines():
        if not line.startswith("$GPGGA,"):
            continue
        gpgga += 1
        parts = line.split("*", 1)[0].split(",")
        if len(parts) < 7:
            continue
        try:
            q = int(parts[6])
        except ValueError:
            continue
        quality_counts[str(q)] = quality_counts.get(str(q), 0) + 1
    fixed = quality_counts.get("4", 0)
    return {
        "solution_gpgga_epochs": gpgga,
        "quality_counts": quality_counts,
        "fix_ratio_gpgga_quality4": fixed / gpgga if gpgga else None,
    }


def parse_node_log(log_path: Path) -> dict[str, Any]:
    if not log_path.is_file():
        return {"error": "missing node.log"}
    text = log_path.read_text(errors="replace")
    crashed = (
        "segment fault" in text.lower()
        or "Check failure stack trace" in text
        or "SIGSEGV" in text
    )
    clean_exit = "signal_handler(SIGINT/SIGTERM)" in text

    rrr_re = re.compile(
        r"RTK/IMU/Camera RRR: Iterations: (\d+), Initial cost: ([0-9.e+-]+), "
        r"Final cost: ([0-9.e+-]+).*Fix status: (\d+)"
    )
    initial_costs: list[float] = []
    final_costs: list[float] = []
    fix_status_counts: dict[str, int] = {}
    for m in rrr_re.finditer(text):
        initial_costs.append(float(m.group(2)))
        final_costs.append(float(m.group(3)))
        st = m.group(4)
        fix_status_counts[st] = fix_status_counts.get(st, 0) + 1

    reject_counts = {
        "pseudorange": len(re.findall(r"Rejected pseudorange outlier", text)),
        "phaserange": len(re.findall(r"Rejected phaserange outlier", text)),
        "doppler": len(re.findall(r"Rejected doppler outlier", text)),
        "landmark": len(re.findall(r"Rejected landmark outlier", text)),
    }
    reject_counts["total"] = sum(reject_counts.values())

    def stats(vals: list[float]) -> dict[str, float | int | None]:
        if not vals:
            return {"n": 0, "mean": None, "std": None, "min": None, "max": None}
        return {
            "n": len(vals),
            "mean": statistics.mean(vals),
            "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
            "min": min(vals),
            "max": max(vals),
        }

    cost_reduction = [
        (i - f) / i for i, f in zip(initial_costs, final_costs) if i > 0
    ]
    return {
        "node_crashed": crashed,
        "node_clean_exit": clean_exit,
        "rrr_update_epochs": len(initial_costs),
        "fix_status_counts": fix_status_counts,
        "fix_ratio_log_status3": fix_status_counts.get("3", 0) / len(initial_costs)
        if initial_costs else None,
        "residual_statistics": {
            "initial_cost": stats(initial_costs),
            "final_cost": stats(final_costs),
            "relative_cost_reduction": stats(cost_reduction),
        },
        "rejected_measurement_count": reject_counts,
    }


def run_once(
    run_idx: int,
    dataset: str,
    bag: Path,
    rate: float,
    provenance: dict[str, Any],
    command_line: list[str],
) -> Path:
    run_dir = REPEAT_ROOT / dataset / f"run_{run_idx:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "log").mkdir(exist_ok=True)
    (run_dir / "output").mkdir(exist_ok=True)

    cfg_path = run_dir / "ros_urbannav_rrr_ros2_adapted.yaml"
    cfg_text = TEMPLATE.read_text().replace("OUTPUT_DIR", str(run_dir))
    cfg_path.write_text(cfg_text)
    config_sha256 = sha256_file(cfg_path)

    kill_stale()
    node_log = run_dir / "node.log"
    solution = run_dir / "solution.txt"
    if solution.exists():
        solution.unlink()

    started_at = utc_now()
    t0 = time.time()
    print(f"[baseline-repeat] run_{run_idx:02d} -> {run_dir}")

    with node_log.open("w") as log_fp:
        node_proc = subprocess.Popen(
            [str(NODE_EXE), str(cfg_path)],
            stdout=log_fp,
            stderr=subprocess.STDOUT,
            cwd=REPO,
        )
        time.sleep(3)
        play = subprocess.run(
            ["ros2", "bag", "play", str(bag),
             "--rate", str(rate), "--read-ahead-queue-size", "10000"],
            check=False,
        )
        drain = int(os.environ.get("RRR_DRAIN_SECONDS", "20"))
        time.sleep(drain)
        node_proc.send_signal(signal.SIGINT)
        for _ in range(60):
            if node_proc.poll() is not None:
                break
            time.sleep(0.5)
        if node_proc.poll() is None:
            node_proc.kill()
    runtime_s = time.time() - t0
    ended_at = utc_now()

    if solution.is_file():
        shutil.copy2(solution, run_dir / "output" / "solution.txt")

    env = os.environ.copy()
    env["GICI_BASELINE_OUT"] = str(run_dir)
    eval_ok = subprocess.run(
        ["bash", str(EVAL_SCRIPT), dataset],
        env=env, cwd=REPO, check=False,
    ).returncode == 0

    fix_stats = parse_solution_fix_ratio(solution)
    log_stats = parse_node_log(node_log)

    ape_metrics: dict[str, Any] = {}
    ape_path = run_dir / "evaluation" / "ape_metrics.json"
    if ape_path.is_file():
        ape_metrics = json.loads(ape_path.read_text())

    gpgga = fix_stats.get("solution_gpgga_epochs", 0)
    run_ok = (
        play.returncode == 0
        and gpgga >= 500
        and not log_stats.get("node_crashed", False)
        and eval_ok
    )
    if not run_ok:
        print(
            f"[baseline-repeat] WARNING incomplete run_{run_idx:02d}: "
            f"gpgga={gpgga} crashed={log_stats.get('node_crashed')} eval_ok={eval_ok}",
            file=sys.stderr,
        )

    binary_sha256 = provenance["binary_sha256"]
    upstream_commit = provenance["upstream_commit"]
    manifest = {
        "run_index": run_idx,
        "dataset": dataset,
        "baseline_tag": BASELINE_TAG,
        "git_sha": provenance["git_sha"],
        "git_clean": provenance["git_clean"],
        "baseline_sha": baseline_sha(config_sha256, binary_sha256, upstream_commit),
        "config_sha256": config_sha256,
        "config_template_sha256": provenance["config_template_sha256"],
        "dataset_sha256": provenance["dataset_sha256"],
        "dataset_files": provenance["dataset_files"],
        "binary_sha256": binary_sha256,
        "binary_mtime_utc": provenance["binary_mtime_utc"],
        "toolchain": provenance["toolchain"],
        "hardware": provenance["hardware"],
        "command_line": command_line,
        "started_at_utc": started_at,
        "ended_at_utc": ended_at,
        "runtime_s": runtime_s,
        "random_seed": None,
        "random_seed_note": "Deterministic offline replay; no explicit RNG seed is configured.",
        "rate": rate,
        "bag_play_exit_code": play.returncode,
        "evaluation_ok": eval_ok,
        "run_ok": run_ok,
        "ape_position_rmse_m": ape_metrics.get("ape_translation_rmse_m"),
        "ape_orientation_rmse_deg": ape_metrics.get("ape_rotation_rmse_deg"),
        "ape_metrics_path": str(ape_path.relative_to(REPO)) if ape_path.is_file() else None,
        "fix_ratio": {
            "gpgga_quality4": fix_stats.get("fix_ratio_gpgga_quality4"),
            "log_fix_status3": log_stats.get("fix_ratio_log_status3"),
            "quality_counts": fix_stats.get("quality_counts"),
            "fix_status_counts": log_stats.get("fix_status_counts"),
        },
        "residual_statistics": log_stats.get("residual_statistics"),
        "rejected_measurement_count": log_stats.get("rejected_measurement_count"),
        "solution_gpgga_epochs": gpgga,
        "rrr_update_epochs": log_stats.get("rrr_update_epochs"),
        "node_log_stats": {
            "node_crashed": log_stats.get("node_crashed"),
            "node_clean_exit": log_stats.get("node_clean_exit"),
        },
    }
    (run_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return run_dir


def summarize_runs(dataset: str, runs: list[Path]) -> dict[str, Any]:
    manifests = []
    for run_dir in runs:
        mpath = run_dir / "run_manifest.json"
        if mpath.is_file():
            manifests.append(json.loads(mpath.read_text()))

    def series(key: str) -> list[float]:
        vals = []
        for m in manifests:
            v = m.get(key)
            if v is not None and not (isinstance(v, float) and math.isnan(v)):
                vals.append(float(v))
        return vals

    def mean_std(vals: list[float]) -> dict[str, Any]:
        if not vals:
            return {"n": 0, "mean": None, "std": None}
        if len(vals) == 1:
            return {"n": 1, "mean": vals[0], "std": 0.0}
        return {"n": len(vals), "mean": statistics.mean(vals), "std": statistics.stdev(vals)}

    summary = {
        "dataset": dataset,
        "n_runs": len(manifests),
        "runs": [str(p.relative_to(REPO)) for p in runs],
        "ape_position_rmse_m": mean_std(series("ape_position_rmse_m")),
        "ape_orientation_rmse_deg": mean_std(series("ape_orientation_rmse_deg")),
        "runtime_s": mean_std(series("runtime_s")),
        "fix_ratio_gpgga_quality4": mean_std([
            float(m["fix_ratio"]["gpgga_quality4"])
            for m in manifests
            if m.get("fix_ratio", {}).get("gpgga_quality4") is not None
        ]),
        "rejected_measurement_total": mean_std([
            float(m["rejected_measurement_count"]["total"])
            for m in manifests
            if m.get("rejected_measurement_count", {}).get("total") is not None
        ]),
        "all_run_ok": all(m.get("run_ok", False) for m in manifests),
    }
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", type=int, default=3, help="Repeat count (default: 3)")
    ap.add_argument("--dataset", default="medium", choices=["medium"])
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--skip-existing", action="store_true")
    args = ap.parse_args()

    if args.runs < 3:
        print("WARNING: fewer than 3 runs requested; natural-variation estimate will be weak.",
              file=sys.stderr)

    if not NODE_EXE.is_file():
        print(f"ERROR: missing binary {NODE_EXE}", file=sys.stderr)
        print("Build with: cd ros2_wrapper && source /opt/ros/humble/setup.bash && colcon build",
              file=sys.stderr)
        return 1

    REPEAT_ROOT.mkdir(parents=True, exist_ok=True)
    bag = ensure_bag(args.dataset)
    dataset_sha256, dataset_files = hash_dataset(args.dataset)

    expected = json.loads(EXPECTED.read_text()) if EXPECTED.is_file() else {}
    upstream_commit = expected.get("upstream_commit", "f2b8579")

    binary_sha256 = sha256_file(NODE_EXE)
    binary_mtime = datetime.fromtimestamp(
        NODE_EXE.stat().st_mtime, tz=timezone.utc).isoformat()

    provenance = {
        "git_sha": git_sha(REPO),
        "git_clean": git_is_clean(REPO),
        "config_template_sha256": sha256_file(TEMPLATE),
        "dataset_sha256": dataset_sha256,
        "dataset_files": dataset_files,
        "binary_sha256": binary_sha256,
        "binary_mtime_utc": binary_mtime,
        "upstream_commit": upstream_commit,
        "toolchain": collect_toolchain(),
        "hardware": collect_hardware(),
    }

    session = {
        "baseline_tag": BASELINE_TAG,
        "started_at_utc": utc_now(),
        "dataset": args.dataset,
        "runs_requested": args.runs,
        "rate": args.rate,
        "provenance": provenance,
        "completed_runs": [],
    }

    completed_dirs: list[Path] = []
    command_line = [sys.executable] + sys.argv

    for run_idx in range(1, args.runs + 1):
        run_dir = REPEAT_ROOT / args.dataset / f"run_{run_idx:02d}"
        if args.skip_existing and (run_dir / "run_manifest.json").is_file():
            print(f"[baseline-repeat] skip existing {run_dir}")
            completed_dirs.append(run_dir)
            continue
        completed_dirs.append(
            run_once(run_idx, args.dataset, bag, args.rate, provenance, command_line)
        )
        session["completed_runs"].append(f"{args.dataset}/run_{run_idx:02d}")

    session["finished_at_utc"] = utc_now()
    session["summary"] = summarize_runs(args.dataset, completed_dirs)
    session_path = REPEAT_ROOT / args.dataset / "session_manifest.json"
    session_path.write_text(json.dumps(session, indent=2) + "\n")

    s = session["summary"]
    print("\n[baseline-repeat] Summary")
    print(f"  runs: {s['n_runs']}")
    if s["ape_position_rmse_m"]["mean"] is not None:
        print(
            f"  APE position  : {s['ape_position_rmse_m']['mean']:.4f} ± "
            f"{s['ape_position_rmse_m']['std']:.4f} m"
        )
    if s["ape_orientation_rmse_deg"]["mean"] is not None:
        print(
            f"  APE rotation  : {s['ape_orientation_rmse_deg']['mean']:.3f} ± "
            f"{s['ape_orientation_rmse_deg']['std']:.3f} deg"
        )
    print(f"  Session manifest: {session_path}")

    if not s.get("all_run_ok", False):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
