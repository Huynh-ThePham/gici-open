#!/usr/bin/env python3
"""Run the UrbanNav RRR RF-BL estimator (bounded-influence GNSS loss on the
plain baseline — attribution arm of research/PREREG_ROBUST_FLOAT.md)."""

from pathlib import Path

import run_urbannav_rrr_baseline as runner


if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RTK/IMU/Camera RRR RF-BL (Tukey GNSS loss)",
            template_path=runner.REPO / "research" / "config" / "rtk_imu_camera_rrr_rfbl_urbannav.yaml",
            expected_estimator="rtk_imu_camera_rrr",
            result_algorithm="rtk_imu_camera_rrr_rfbl",
            default_out_root=Path(runner.REPO) / "results" / "research" / "urbannav_rfbl",
        )
    )
