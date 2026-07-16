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