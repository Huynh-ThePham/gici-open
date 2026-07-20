#!/usr/bin/env python3
"""UrbanNav Ceres-oracle AR (exact joint Q_aa into LAMBDA). Slow offline."""
from pathlib import Path
import run_urbannav_rrr_baseline as runner

if __name__ == "__main__":
    raise SystemExit(
        runner.main(
            description="UrbanNav RRR-VA Ceres covariance oracle",
            template_path=runner.REPO
            / "research/config/rtk_imu_camera_rrr_va_urbannav_ceres_oracle.yaml",
            expected_estimator="rtk_imu_camera_rrr_va",
            result_algorithm="rtk_imu_camera_rrr_va_ceres",
            default_out_root=Path(runner.REPO) / "results" / "research" / "ceres_oracle",
        )
    )
