#!/usr/bin/env python3
"""Run the UrbanNav RRR-VA research estimator.

This is intentionally separate from run_urbannav_rrr_baseline.py. Outputs default to
results/research/urbannav_va so experimental runs cannot overwrite locked baseline
artifacts.
"""

from pathlib import Path

import run_urbannav_rrr_baseline as runner


if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RTK/IMU/Camera RRR-VA research estimator",
            template_path=runner.RESEARCH_VA_TEMPLATE,
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_va",
            default_out_root=Path(runner.REPO) / "results" / "research" / "urbannav_va",
        )
    )
