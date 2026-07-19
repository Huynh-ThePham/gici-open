/**
* @Function: RTK/IMU/Camera tightly couple estimator (GNSS raw (RTK formula) + IMU raw + camera raw)
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_estimator_base.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/vision/visual_estimator_base.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// RTK/IMU/Camera RRR couple options
struct RtkImuCameraRrrEstimatorOptions {
  // Frame state window length
  // We only keep GNSS measurements near to keyframes (one-to-one) and throw the others 
  // away after one optimization, because the GNSS measurement errors, especially for 
  // the multipath, are highly correlated between epochs when we have a slow or zero motion.
  // Besides, we need at least 2 GNSS states in window. If current setting cannot ensure 
  // this condition, we will ignore this option and extend the windows length.
  int max_keyframes = 5;

  // GNSS state window length before visual has been initialized
  int max_gnss_window_length_minor = 3;

  // Maximum yaw STD to start visual initialization (deg)
  double min_yaw_std_init_visual = 0.5;

  // Vision-aided ambiguity resolution research (research/vision-aided-ambiguity-
  // resolution branch): when true, additionally computes the ambiguity covariance
  // directly from the real joint graph (paired with the current epoch's pose/IMU
  // state, not the full sliding window) and logs a timing comparison against the
  // existing GNSS-only shadow-estimator path. Purely diagnostic -- does not affect
  // the AR decision or any estimator output. Default off so existing locked
  // baselines are unaffected.
  bool benchmark_joint_ambiguity_covariance = false;

  // Vision-aided ambiguity resolution research switch:
  // in the base RtkImuCameraRrrEstimator this tries a current-epoch local joint
  // covariance over ambiguities + pose/speed-and-bias and falls back to the
  // GNSS-only shadow covariance on rank failure. The separate
  // RtkImuCameraRrrVaEstimator overrides this path. Both are opt-in research
  // paths, not locked-baseline semantics; see research/VISION_AIDED_AR.md.
  bool use_vision_aided_ambiguity_resolution = false;

  // Exact joint-graph AR covariance for RtkImuCameraRrrVaEstimator. When true,
  // VA requests covariance only for the current ambiguity blocks from the real
  // Ceres problem. Ceres then marginalizes every other active graph variable
  // (poses, speed/bias, landmarks, extrinsics, clocks/frequencies, marginalization
  // priors) consistently through the full problem Jacobian. This is the statistically
  // clean path, but it is much slower than the bounded local-information experiments.
  bool ar_use_exact_joint_covariance = true;

  // Vision-ablation control (research/vision-aided-ambiguity-resolution): only used by
  // the RtkImuCameraRrrVaEstimator. When true, the camera-keyframe (cPose) chain is
  // still added to the local cross-information, but its reprojection residuals are
  // withheld -- so the camera pose is present but carries no visual constraint. This
  // isolates whether the va estimator's accuracy gain comes from genuine visual
  // information (gain disappears when reprojection is withheld) or merely from adding
  // more marginalized blocks (gain persists). Diagnostic only; default off.
  bool ablate_reprojection_in_ar = false;

  // Legacy vision-aided AR covariance mix. Diagnostic only: Cov = gamma * local +
  // (1 - gamma) * shadow. This is not statistically consistent and is retained only
  // for ablation/reproducibility of failed experiments.
  double vision_ar_covariance_mix = 0.5;

  // Legacy experimental delta-information fusion:
  //     I_final = I_shadow + (A_with_reprojection - A_without_reprojection)
  // where A_* is the marginal ambiguity information from local matrices with/without
  // reprojection residuals. This avoids reusing the same GNSS residual in the explicit
  // shadow term, but it is still an approximation: unrequested neighboring states and
  // landmarks are conditioned at their current values rather than marginalized. Repeated
  // UrbanNav Deep runs show this can increase fixed-rate while degrading horizontal
  // accuracy. It is kept only for ablation when ar_use_exact_joint_covariance=false.
  bool ar_use_delta_information = true;

  // Fast exact-in-window marginal ambiguity covariance for RtkImuCameraRrrVaEstimator.
  // Computes the SAME quantity as ar_use_exact_joint_covariance -- Q_aa = [H_active^-1]_aa,
  // the marginal covariance of the current ambiguity blocks after marginalizing (not
  // conditioning) every other active state including the marginalization prior -- but by
  // exploiting the factor-graph sparsity (block-eliminate landmarks, then dense-eliminate
  // the remaining nuisance states) instead of ceres::Covariance's whole-problem QR/SVD.
  // Cost is bounded by the sliding window, not the trajectory length, so the statistically
  // consistent covariance becomes usable at every AR epoch in real time. Returns false
  // (caller falls back to the plain shadow covariance) when the reduced ambiguity
  // information is not positive-definite -- never fabricating confidence. Precedence in
  // the VA estimator: fast (this) -> exact (ceres) -> delta/mix. See
  // research/VISION_AIDED_AR.md. Default on: this is the intended VA path.
  bool ar_use_fast_marginal_covariance = true;
};

// Estimator
class RtkImuCameraRrrEstimator : 
  public GnssEstimatorBase, 
  public VisualEstimatorBase, 
  public ImuEstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RtkImuCameraRrrEstimator(const RtkImuCameraRrrEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const RtkEstimatorOptions rtk_options,
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const VisualEstimatorBaseOptions& visual_base_options,
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options);
  ~RtkImuCameraRrrEstimator();

  // Add measurement
  bool addMeasurement(const EstimatorDataCluster& measurement) override;

  // Estimate current graph
  bool estimate() override;

  // Set initializatin result
  void setInitializationResult(
    const std::shared_ptr<MultisensorInitializerBase>& initializer) override;

protected:
  // Add GNSS measurements and state
  bool addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref);

  // Add image measurements and state
  bool addImageMeasurementAndState(const FrameBundlePtr& frame_bundle, 
    const SpeedAndBias& speed_and_bias = SpeedAndBias::Zero());

  // Visual initialization
  bool visualInitialization(const FrameBundlePtr& frame_bundle);

  // Marginalization
  bool marginalization(const IdType& type);

  // Marginalization when the new state is a frame state
  bool frameMarginalization();

  // Marginalization when the new state is a GNSS state
  bool gnssMarginalization();

  // Sparsify GNSS states to bound computational load
  void sparsifyGnssStates();

  // Compute ambiguity covariance at current epoch
  bool estimateAmbiguityCovariance(const State& state, Eigen::MatrixXd& covariance);

  // Research diagnostic (see benchmark_joint_ambiguity_covariance option above):
  // times computing the ambiguity covariance directly from the real joint graph_
  // (ambiguity blocks + current epoch's pose/IMU block only, not the full window)
  // and logs it against the existing shadow-estimator path's timing.
  void benchmarkJointAmbiguityCovariance(const State& state);

  // Vision-aided ambiguity resolution (see use_vision_aided_ambiguity_resolution
  // option above): fuses the shadow-estimator's ambiguity covariance with the
  // current epoch's local cross-information against the tightly-coupled pose/
  // speed-and-bias state. Returns false (leaving `covariance` untouched) if either
  // the shadow covariance or the fused information matrix is unusable, so the
  // caller can fall back to the plain shadow covariance.
  // Virtual so the RtkImuCameraRrrVaEstimator subclass (research/vision-aided-
  // ambiguity-resolution) can extend the local cross-information parameter set to
  // include the current camera keyframe (cPose), routing genuine visual information
  // into the ambiguity covariance rather than only the IMU/pose chain.
  virtual bool estimateVisionAidedAmbiguityCovariance(
    const State& state, Eigen::MatrixXd& covariance);

  // Get latest state
  inline State& latestState() override { return states_[latest_state_index_]; }

protected:
  // Options
  RtkImuCameraRrrEstimatorOptions rrr_options_;
  RtkEstimatorOptions rtk_options_;

  // Initialization control
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer_;
  std::shared_ptr<RtkEstimator> initializer_sub_estimator_;
  bool visual_initialized_ = false;
  std::deque<FrameBundlePtr> init_keyframes_;
  std::deque<Solution> init_solution_store_;

  // Measurement alignment handle
  DifferentialMeasurementsAlign meausrement_align_;

  // RTK estimator used for ambiguity covariance estimation
  std::unique_ptr<RtkEstimator> ambiguity_covariance_estimator_;
  bool ambiguity_covariance_coordinate_setted_ = false;

  // Status control
  int num_continuous_unfix_ = 0;
  int num_cotinuous_reject_gnss_ = 0;
  int num_cotinuous_reject_visual_ = 0;
};

}
