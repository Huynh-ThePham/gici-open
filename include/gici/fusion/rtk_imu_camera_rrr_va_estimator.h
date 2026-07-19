/**
* @Function: RTK/IMU/Camera RRR estimator with genuinely vision-aided ambiguity
*             resolution (research/vision-aided-ambiguity-resolution).
*
* This is a thin subclass of RtkImuCameraRrrEstimator. It changes exactly one thing:
* the ambiguity-resolution covariance is computed from the real joint graph by asking
* Ceres for the marginal covariance of the current ambiguity blocks. All other active
* graph variables -- poses, speed/bias, camera keyframes, visual landmarks, extrinsics,
* clocks/frequencies, and marginalization priors -- are nuisance variables eliminated
* by the full problem covariance calculation. Therefore reprojection residuals affect
* AR through the same joint graph used by the estimator, without the previous local
* conditioning or shadow+delta approximation.
*
* Everything else (state management, marginalization, sensor handling) is inherited
* unchanged. The original RtkImuCameraRrr estimator/type is left byte-untouched, so
* the frozen baselines are unaffected -- this is a separately registered estimator
* type (`rtk_imu_camera_rrr_va`).
**/
#pragma once

#include "gici/fusion/rtk_imu_camera_rrr_estimator.h"

namespace gici {

class RtkImuCameraRrrVaEstimator : public RtkImuCameraRrrEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RtkImuCameraRrrVaEstimator(const RtkImuCameraRrrEstimatorOptions& options,
               const GnssImuInitializerOptions& init_options,
               const RtkEstimatorOptions rtk_options,
               const GnssEstimatorBaseOptions& gnss_base_options,
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
               const VisualEstimatorBaseOptions& visual_base_options,
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options);
  ~RtkImuCameraRrrVaEstimator();

protected:
  // Genuinely vision-aided ambiguity covariance. Path precedence (all compute the marginal
  // ambiguity covariance from the real joint graph, so reprojection/IMU information enters
  // AR): fast structure-exploiting Schur (ar_use_fast_marginal_covariance, default) ->
  // exact whole-problem ceres::Covariance (ar_use_exact_joint_covariance) -> legacy local
  // delta/mix ablation. The fast and exact paths are the SAME quantity Q_aa = [H^-1]_aa
  // (marginalizing every non-ambiguity active state incl. the marginalization prior); the
  // fast path just exploits factor-graph sparsity so it is real-time at every AR epoch.
  // Returns false (caller falls back to the plain GNSS-only shadow covariance) when the
  // covariance is not usable -- never fabricating confidence.
  bool estimateVisionAidedAmbiguityCovariance(
    const State& state, Eigen::MatrixXd& covariance) override;
};

}  // namespace gici
