#!/usr/bin/env python3
"""Run the UrbanNav RRR-VA estimator with vision ABLATED (ablate_reprojection_in_ar=true).

Vision-off control for the vision-aided-AR claim. Same estimator and same fast-marginal
covariance machinery as run_urbannav_rrr_va.py, but the camera reprojection residuals are
dropped from the ambiguity marginal (config rtk_imu_camera_rrr_va_noreproj_urbannav.yaml).
Comparing this against the vision-on VA run isolates the pure vision contribution and closes
the proposed-vs-shadow confound (full-graph marginal with IMU is held fixed; only vision toggles).
Outputs default to results/research/urbannav_va_noreproj so no locked artifact is overwritten.
"""

from pathlib import Path

import run_urbannav_rrr_baseline as runner


if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RTK/IMU/Camera RRR-VA, vision ablated (reprojection off in AR)",
            template_path=runner.REPO
            / "research"
            / "config"
            / "rtk_imu_camera_rrr_va_noreproj_urbannav.yaml",
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_va",
            default_out_root=Path(runner.REPO) / "results" / "research" / "urbannav_va_noreproj",
        )
    )
