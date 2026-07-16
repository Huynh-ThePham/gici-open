#!/usr/bin/env python3
"""Provision a fixed UrbanNav dataset layout for GICI file-mode RRR.

Layout (under <dataset-root>/):
  original/     — raw official inputs (symlinks, no duplication)
  calibration/  — extrinsics / intrinsics / IMU noise
  gici_rrr/     — canonical GICI post-file inputs + resolved_config.yaml
  reports/      — run outputs, validation manifests

Usage:
  python3 scripts/provision_urbannav_layout.py medium [--root PATH] [--force]
  python3 scripts/provision_urbannav_layout.py deep   [--root PATH] [--force]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import struct
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
TEMPLATE = REPO / "research" / "config" / "rtk_imu_camera_rrr_urbannav.yaml"

IMG_PREAMB = bytes([0xFE, 0xCA, 0x00, 0xFF, 0x00, 0xFF])
IMG_TAIL = bytes([0xCC, 0xFE, 0xFF, 0x00, 0xFF, 0x00])

_DEFAULT_DATA_ROOT = Path("/media/theph/Data1/Research/dataset/UrbanNavDataset")


@dataclass(frozen=True)
class SceneSpec:
    key: str
    dirname: str
    sensors_bag: str
    gt_file: str
    imu_csv: str
    rover_obs: str
    ref_obs: str
    eph_nav: str
    dcb_rel: str
    cam_w: int
    cam_h: int
    cam_step: int


SCENES: dict[str, SceneSpec] = {
    "medium": SceneSpec(
        key="medium",
        dirname="UrbanNav-HK-Medium-Urban-1",
        sensors_bag="ros/UrbanNav-HK_TST-20210517_sensors.bag",
        gt_file="UrbanNav_TST_GT_raw.txt",
        imu_csv="imu/xsense_imu_medium_urban1.csv",
        rover_obs="gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs",
        ref_obs="gnss/base/hkkt137g.rnx",
        eph_nav="gnss/base/brdc1370.rnx",
        dcb_rel="research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX",
        cam_w=672,
        cam_h=376,
        cam_step=1,
    ),
    "deep": SceneSpec(
        key="deep",
        dirname="UrbanNav-HK-Deep-Urban-1",
        sensors_bag="ros/UrbanNav-HK_Whampoa-20210521_sensors.bag",
        gt_file="UrbanNav_whampoa_raw.txt",
        imu_csv="imu/xsense_imu_deep_urban1.csv",
        rover_obs="gnss/UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs",
        ref_obs="gnss/base/_deprecated/hkkt141g.21o",
        eph_nav="gnss/base/_deprecated/brdc_mn.rnx",
        dcb_rel="research/dcb/CAS0MGXRAP_20211410000_01D_01D_DCB.BSX",
        cam_w=672,
        cam_h=376,
        cam_step=1,
    ),
}


def _getbitu(data: bytes, pos: int, length: int) -> int:
    val = 0
    for i in range(length):
        byte_idx = (pos + i) // 8
        bit_idx = 7 - ((pos + i) % 8)
        if byte_idx < len(data):
            val = (val << 1) + ((data[byte_idx] >> bit_idx) & 1)
    return val


def extract_image_timestamps(camera_bin: Path, width: int, height: int, step: int) -> list[tuple[float, int]]:
    """Parse GICI image-pack and return (unix_time, frame_index) rows."""
    rows: list[tuple[float, int]] = []
    expected_payload = 13 + width * height * step + 6  # header + pixels + tail

    with camera_bin.open("rb") as fp:
        hdr = fp.read(9)
        if len(hdr) < 9 or hdr[:6] != IMG_PREAMB:
            raise ValueError(f"invalid image-pack header in {camera_bin}")
        payload_len = _getbitu(hdr, 48, 24)
        if payload_len != expected_payload:
            raise ValueError(
                f"unexpected payload length {payload_len} (expected {expected_payload})"
            )
        stride = 9 + payload_len
        fp.seek(0)

        frame_idx = 0
        while True:
            hdr = fp.read(9)
            if len(hdr) < 9:
                break
            if hdr[:6] != IMG_PREAMB:
                break
            if _getbitu(hdr, 48, 24) != payload_len:
                break
            payload = fp.read(payload_len)
            if len(payload) < payload_len or payload[-6:] != IMG_TAIL:
                break
            w = struct.unpack_from(">H", payload, 8)[0]
            h = struct.unpack_from(">H", payload, 10)[0]
            if w != width or h != height or payload[12] != step:
                break
            t_sec, t_frac = struct.unpack_from(">II", payload, 0)
            rows.append((float(t_sec) + t_frac * 1e-9, frame_idx))
            frame_idx += 1

    return rows


def md5_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    h = hashlib.md5()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def link_or_copy(src: Path, dst: Path, force: bool) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        if not force:
            return
        dst.unlink()
    if not src.is_file():
        raise FileNotFoundError(src)
    try:
        os.symlink(src.resolve(), dst)
    except OSError:
        shutil.copy2(src, dst)


def render_resolved_config(scene: SceneSpec, root: Path, dcb: Path) -> str:
    gici = root / "gici_rrr"
    text = TEMPLATE.read_text()
    cam_buffer = scene.cam_w * scene.cam_h + 512
    repl = {
        "<ROVER_OBS>": str(gici / "gnss_rover.obs"),
        "<REF_OBS>": str(gici / "gnss_reference.obs"),
        "<EPH_NAV>": str(gici / "gnss_ephemeris.nav"),
        "<DCB_FILE>": str(dcb),
        "<IMU_FILE>": str(gici / "imu.txt"),
        "<CAMERA_FILE>": str(gici / "camera.bin"),
        "<CAM_BUFFER>": str(cam_buffer),
        "<OUTPUT_DIR>": str(root / "reports" / "output"),
    }
    for k, v in repl.items():
        text = text.replace(k, v)
    return text


def provision(scene_key: str, root: Path, force: bool) -> dict:
    scene = SCENES[scene_key]
    if root.name != scene.dirname:
        root = root / scene.dirname

    dcb = REPO / scene.dcb_rel
    if not root.is_dir():
        raise FileNotFoundError(f"dataset root missing: {root}")
    if not dcb.is_file():
        raise FileNotFoundError(f"DCB missing: {dcb}")
    if not TEMPLATE.is_file():
        raise FileNotFoundError(f"config template missing: {TEMPLATE}")

    original = root / "original"
    calibration = root / "calibration"
    gici_rrr = root / "gici_rrr"
    reports = root / "reports"
    for d in (original, calibration, gici_rrr, reports, reports / "output"):
        d.mkdir(parents=True, exist_ok=True)

    # --- original/ ---
    src_map = {
        original / "sensors.bag": root / scene.sensors_bag,
        original / "UrbanNav_TST_GT_raw.txt": root / scene.gt_file,
        original / scene.imu_csv.split("/")[-1]: root / scene.imu_csv,
    }
    if scene.key == "deep":
        src_map[original / "UrbanNav_whampoa_raw.txt"] = root / scene.gt_file
        src_map.pop(original / "UrbanNav_TST_GT_raw.txt", None)

    for dst, src in src_map.items():
        link_or_copy(src, dst, force)

    gnss_dst = original / "gnss"
    if gnss_dst.exists() or gnss_dst.is_symlink():
        if force:
            gnss_dst.unlink()
    if not gnss_dst.exists():
        os.symlink((root / "gnss").resolve(), gnss_dst)

    # --- calibration/ ---
    for name in ("extrinsic.yaml", "zed2_intrinsics.yaml", "xsens_imu_param.yaml"):
        link_or_copy(root / name, calibration / name, force)

    # --- gici_rrr/ GNSS canonical names ---
    gnss_links = {
        "gnss_rover.obs": root / scene.rover_obs,
        "gnss_reference.obs": root / scene.ref_obs,
        "gnss_ephemeris.nav": root / scene.eph_nav,
    }
    for name, src in gnss_links.items():
        link_or_copy(src, gici_rrr / name, force)

    # IMU text: prefer existing conversion, fall back to regenerating header-only stub
    imu_candidates = [
        gici_rrr / "imu.bin.txt",
        root / "gici_rrr_backup_20260709" / "imu.bin.txt",
    ]
    imu_src = next((p for p in imu_candidates if p.is_file()), None)
    if imu_src is None:
        raise FileNotFoundError(
            f"No imu.bin.txt found under {gici_rrr}; convert sensors.bag first."
        )
    link_or_copy(imu_src, gici_rrr / "imu.txt", force)

    # camera.bin: keep in place or link from backup
    if not (gici_rrr / "camera.bin").is_file():
        cam_src = next(
            (p for p in (gici_rrr / "camera.bin", root / "gici_rrr_backup_20260709" / "camera.bin") if p.is_file()),
            None,
        )
        if cam_src and cam_src != gici_rrr / "camera.bin":
            link_or_copy(cam_src, gici_rrr / "camera.bin", force)

    if not (gici_rrr / "camera.bin").is_file():
        raise FileNotFoundError(f"camera.bin missing under {gici_rrr}")

    # image_timestamps.csv
    ts_path = gici_rrr / "image_timestamps.csv"
    if force or not ts_path.is_file():
        rows = extract_image_timestamps(
            gici_rrr / "camera.bin", scene.cam_w, scene.cam_h, scene.cam_step
        )
        with ts_path.open("w") as fp:
            fp.write("frame_index,timestamp\n")
            for ts, idx in rows:
                fp.write(f"{idx},{ts:.9f}\n")

    # resolved_config.yaml (author wrapper template, canonical paths)
    cfg_path = gici_rrr / "resolved_config.yaml"
    if force or not cfg_path.is_file():
        cfg_path.write_text(render_resolved_config(scene, root, dcb))

    manifest = {
        "schema": "urbannav-layout-v1",
        "scene": scene.key,
        "root": str(root.resolve()),
        "provisioned_at": datetime.now(timezone.utc).isoformat(),
        "layout": {
            "original": {
                "sensors.bag": str((original / "sensors.bag").resolve()),
                "gnss": str((original / "gnss").resolve()),
                "ground_truth": str((original / scene.gt_file.split("/")[-1]).resolve()),
                "imu_csv": str((original / scene.imu_csv.split("/")[-1]).resolve()),
            },
            "calibration": str(calibration.resolve()),
            "gici_rrr": {
                "gnss_rover.obs": str((gici_rrr / "gnss_rover.obs").resolve()),
                "gnss_reference.obs": str((gici_rrr / "gnss_reference.obs").resolve()),
                "gnss_ephemeris.nav": str((gici_rrr / "gnss_ephemeris.nav").resolve()),
                "imu.txt": str((gici_rrr / "imu.txt").resolve()),
                "camera.bin": str((gici_rrr / "camera.bin").resolve()),
                "image_timestamps.csv": str(ts_path.resolve()),
                "resolved_config.yaml": str(cfg_path.resolve()),
            },
            "reports": str(reports.resolve()),
        },
        "checksums": {
            "camera.bin.md5": md5_file(gici_rrr / "camera.bin"),
            "imu.txt.md5": md5_file(gici_rrr / "imu.txt"),
            "gnss_rover.obs.md5": md5_file(gici_rrr / "gnss_rover.obs"),
        },
        "counts": {
            "camera_frames": sum(1 for _ in open(ts_path)) - 1 if ts_path.is_file() else 0,
            "imu_lines": sum(1 for _ in open(gici_rrr / "imu.txt")) - 1,
        },
        "gici_run": {
            "command": f"{REPO / 'build' / 'gici_main'} {gici_rrr / 'resolved_config.yaml'}",
            "output": str((reports / "output" / "solution.txt").resolve()),
        },
    }
    manifest_path = reports / "layout_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", choices=sorted(SCENES))
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(os.environ.get("URBANNAV_DATA_ROOT", str(_DEFAULT_DATA_ROOT))),
        help="UrbanNav data root (parent of UrbanNav-HK-*-1/)",
    )
    parser.add_argument("--force", action="store_true", help="overwrite generated files")
    args = parser.parse_args()

    try:
        manifest = provision(args.scene, args.root, args.force)
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(manifest, indent=2))
    print(f"\nOK: layout provisioned under {manifest['root']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
