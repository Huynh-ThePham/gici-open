#!/usr/bin/env python3
"""Run the UrbanNav RRR RF-CVA estimator (SOFT Cauchy GNSS loss + VA-v4).

PREREG_ROBUST_FLOAT.md addendum: exploratory soft bounded-influence variant
(Cauchy c=2.3849) to preempt "did a gentler robust loss help?". One flag
difference from the RF-VA template (use_cauchy_gnss_loss instead of
use_bounded_influence_gnss_loss)."""

from pathlib import Path

import run_urbannav_rrr_baseline as runner


if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RTK/IMU/Camera RRR RF-CVA (Cauchy GNSS loss)",
            template_path=runner.REPO / "research" / "config" / "rtk_imu_camera_rrr_rfcva_urbannav_iso.yaml",
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_rfcva_iso",
            default_out_root=Path(runner.REPO) / "results" / "research" / "urbannav_rfcva_iso",
        )
    )
