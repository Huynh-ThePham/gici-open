#!/usr/bin/env python3
"""Run the UrbanNav RRR-VA-v3 ("revocable fixes") research estimator.

Pre-registered protocol: research/PREREG_SOFT_FIX.md. The config differs from the
VA-v2 template by exactly one flag (margin_ambiguity_fix_constraints: false).
Outputs default to results/research/urbannav_va3 so experimental runs cannot
overwrite locked baseline artifacts.
"""

from pathlib import Path

import run_urbannav_rrr_baseline as runner


if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RTK/IMU/Camera RRR-VA-v3 (revocable fixes)",
            template_path=runner.REPO / "research" / "config" / "rtk_imu_camera_rrr_va3_urbannav.yaml",
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_va3",
            default_out_root=Path(runner.REPO) / "results" / "research" / "urbannav_va3",
        )
    )
