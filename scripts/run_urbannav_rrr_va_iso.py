#!/usr/bin/env python3
"""UrbanNav float-path isolation arm (fast Q_aa, deterministic solver)."""
from pathlib import Path
import run_urbannav_rrr_baseline as runner

if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RRR-VA isolation (deterministic, fast cov)",
            template_path=runner.REPO / "research/config/rtk_imu_camera_rrr_va_urbannav_iso.yaml",
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_va_iso",
            default_out_root=Path(runner.REPO) / "results" / "research" / "iso_medium" / "va",
        )
    )
