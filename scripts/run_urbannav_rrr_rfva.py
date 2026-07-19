#!/usr/bin/env python3
"""Run the UrbanNav RRR RF-VA estimator (bounded-influence GNSS loss + VA-v4).

Pre-registered protocol: research/PREREG_ROBUST_FLOAT.md. The config differs
from the VA-v4 template by exactly one flag (use_bounded_influence_gnss_loss).
"""

from pathlib import Path

import run_urbannav_rrr_baseline as runner


if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RTK/IMU/Camera RRR RF-VA (Tukey GNSS loss)",
            template_path=runner.REPO / "research" / "config" / "rtk_imu_camera_rrr_rfva_urbannav.yaml",
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_rfva",
            default_out_root=Path(runner.REPO) / "results" / "research" / "urbannav_rfva",
        )
    )
