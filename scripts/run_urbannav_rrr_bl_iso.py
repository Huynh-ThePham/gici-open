#!/usr/bin/env python3
"""UrbanNav float-path isolation arm (shadow AR, deterministic solver)."""
from pathlib import Path
import run_urbannav_rrr_baseline as runner

if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RRR baseline isolation (deterministic)",
            template_path=runner.REPO / "research/config/rtk_imu_camera_rrr_urbannav_iso.yaml",
            expected_estimator="rtk_imu_camera_rrr",
            result_algorithm="rtk_imu_camera_rrr_iso",
            default_out_root=Path(runner.REPO) / "results" / "research" / "iso_medium" / "bl",
        )
    )
