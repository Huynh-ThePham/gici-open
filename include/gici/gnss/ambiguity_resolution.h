/**
* @Function: Ambiguity Resolution
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/utility/common.h"
#include "gici/gnss/gnss_types.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/graph.h"
#include "gici/gnss/ambiguity_common.h"

namespace gici {

// Ambiguity resolution options
struct AmbiguityResolutionOptions {
  // Usage of satellite systems
  // Currently we do not support GLONASS ambiguity resolution
  std::vector<char> system_exclude = {'R'};

  // Usage of specific satellite
  // In default, we use all satellites
  std::vector<std::string> satellite_exclude;

  // Usage of phase types
  // In default, we use all phase types
  std::vector<std::pair<char, int>> phase_exclude;

  // Minimum elevation angle (deg)
  double min_elevation = 15.0;

  // Percentage of narrow lane ambiguity fixation to consider as fixed solution
  double min_percentage_fixation_nl = 0.9;

  // Percentage of wide lane ambiguity fixation to consider as successed
  double min_percentage_fixation_wl = 0.9;

  // Percentage of ultra wide lane ambiguity fixation to consider as successed
  double min_percentage_fixation_uwl = 1.0;

  // Minimum number of satellite pairs for valid ambiguity resolution
  int min_num_satellite_pairs_fixation = 6;

  // Ambiguity fixation ratio for LAMBDA
  double ratio = 3.0;

  // research/vision-aided-ambiguity-resolution (opt-in; default off preserves upstream
  // behavior byte-identically):
  //
  // Joint post-fix validation: extend the existing accept rule (reject when the GNSS
  // range cost increases after applying the fixation constraints) to ALSO reject when
  // the joint vision+IMU cost (reprojection + IMU preintegration residuals, robustified,
  // excluding the fixation constraints themselves) increases. Zero-threshold, exactly
  // mirroring the upstream rule -- no new tuned constants. A subtly-wrong integer that
  // still matches GNSS ranges bends the trajectory against vision/IMU and gets vetoed
  // at acceptance time.
  bool use_joint_cost_validation = false;

  // Integer-bootstrapping success-rate gate: only attempt/accept a (partial) fixation
  // subset whose theoretical success rate P_s = prod(2*Phi(1/(2*sigma_i)) - 1)
  // (Teunissen integer bootstrapping, computed on the LAMBDA-decorrelated conditional
  // variances of the float covariance) is at least this value. 0 disables (upstream
  // behavior). This is a probabilistic specification (failure-rate budget), declared
  // a priori -- not a threshold fitted to any dataset.
  double min_bootstrap_success_rate = 0.0;

  // Decision-confidence-weighted fix constraints (research/PREREG_SOFT_WEIGHT.md):
  // replace the hard-coded constraint information (1e6 cycle^-2, std 0.001 cycles)
  // with information(P_s) = 1 / [(1 - P_s) * 1 + P_s * 1e-6] cycle^-2, the Gaussian
  // moment match of the integer decision mixture (correct w.p. P_s with upstream's
  // baseline variance; wrong w.p. 1-P_s with ~1-cycle error). No free parameters:
  // P_s is the bootstrapping success rate of the exact accepted subset. Default
  // false = upstream byte-identical.
  bool use_success_rate_fix_information = false;
};

// Ambiguity resolution (AR)
class AmbiguityResolution {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Combination types
  enum class LaneType {
    NL, WL, UWL
  };

  // Results
  enum class Result {
    NoFix, WlFix, NlFix
  };

  // Ambiguity deletion method for partial AR
  enum class DelMethod {
    Fractional, Elevation, Variance
  };

  // Ambiguity 
  struct Spec {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    std::string prn;
    double value;  // in meter
    double wavelength;
    double elevation;
    double std;
    std::shared_ptr<ParameterBlock> parameter_block;
    Graph::ResidualBlockSpec residual_block;
    bool is_reference = false;
  };

  // Between Satellite Difference (BSD) ambiguity pair
  struct BsdPair {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BsdPair() { }
    BsdPair(std::vector<Spec>& ambiguities, size_t i, size_t i_ref) : 
      spec_id(i), spec_id_ref(i_ref) {
      value = ambiguities[i].value - ambiguities[i_ref].value;
      wavelength = ambiguities[i].wavelength;
      elevation = ambiguities[i].elevation;
      std = sqrt(square(ambiguities[i].std) + square(ambiguities[i_ref].std));
    }

    // update values after Spec change
    void update(std::vector<Spec>& ambiguities) {
      value = ambiguities[spec_id].value - ambiguities[spec_id_ref].value;
      std = sqrt(square(ambiguities[spec_id].std) + 
            square(ambiguities[spec_id_ref].std));
    }

    size_t spec_id;
    size_t spec_id_ref;
    double value;   // in meter
    double wavelength;
    double elevation; // elevation of the spec_id satellite
    double std; 
    bool is_fixed = false;
    bool is_base_frequency = false;
  };

  // Ultra-widelane (UWL), Widelane (WL) and Narrowlane (NL) ambiguity pair
  struct LanePair {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LanePair() { }
    // For UWL and WL
    LanePair(std::vector<BsdPair>& ambiguity_pairs, size_t i_higher, 
      size_t i_lower) : bsd_pair_id_higher(i_higher), 
      bsd_pair_id_lower(i_lower) {
      wavelength = 1.0 / (1.0 / ambiguity_pairs[i_higher].wavelength - 
                   1.0 / ambiguity_pairs[i_lower].wavelength);
      value = (ambiguity_pairs[i_higher].value / 
               ambiguity_pairs[i_higher].wavelength - 
               ambiguity_pairs[i_lower].value / 
               ambiguity_pairs[i_lower].wavelength) * 
               wavelength;
      elevation = ambiguity_pairs[i_higher].elevation;
      std = sqrt(square(ambiguity_pairs[i_higher].std / 
                 ambiguity_pairs[i_higher].wavelength) + 
                 square(ambiguity_pairs[i_lower].std / 
                 ambiguity_pairs[i_lower].wavelength)) * 
                 wavelength;
    }
    // For NL (stand-alone frequency)
    LanePair(std::vector<BsdPair>& ambiguity_pairs, size_t i) :
      bsd_pair_id_lower(1e6), bsd_pair_id_higher(i) {
      wavelength = ambiguity_pairs[i].wavelength;
      value = ambiguity_pairs[i].value;
      elevation = ambiguity_pairs[i].elevation;
      std = ambiguity_pairs[i].std;
    }

    // Distinguish NL, WL and UWL
    LaneType laneType() {
      if (wavelength < 0.3) return LaneType::NL;
      else if (wavelength < 2.0) return LaneType::WL;
      else return LaneType::UWL;
    }

    // update values after Spec change
    void update(std::vector<BsdPair>& ambiguity_pairs) {
      if (laneType() == LaneType::NL) {
        value = ambiguity_pairs[bsd_pair_id_higher].value;
        std = ambiguity_pairs[bsd_pair_id_higher].std;
      }
      else {
        value = (ambiguity_pairs[bsd_pair_id_higher].value / 
                 ambiguity_pairs[bsd_pair_id_higher].wavelength - 
                 ambiguity_pairs[bsd_pair_id_lower].value / 
                 ambiguity_pairs[bsd_pair_id_lower].wavelength) * 
                 wavelength;
        std = sqrt(square(ambiguity_pairs[bsd_pair_id_higher].std / 
                  ambiguity_pairs[bsd_pair_id_higher].wavelength) + 
                  square(ambiguity_pairs[bsd_pair_id_lower].std / 
                  ambiguity_pairs[bsd_pair_id_lower].wavelength)) * 
                  wavelength;
      }
    }

    size_t bsd_pair_id_lower;   // with lower frequency
    size_t bsd_pair_id_higher;  // with higher frequency
    double value;  // in meter
    double wavelength;
    double elevation;
    double std;
    bool is_fixed = false;
    int num_consistant = 0;
    ceres::ResidualBlockId residual_id = nullptr;
  };

  // Other parameters
  struct Parameter {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackendId id;
    Eigen::VectorXd value;
    size_t size;
    size_t minimal_size;
    size_t covariance_start_index;
    std::shared_ptr<ParameterBlock> handle;
  };

public:
  AmbiguityResolution(const AmbiguityResolutionOptions options,
                      const std::shared_ptr<Graph>& graph);
  ~AmbiguityResolution();

  // For Precise Point Positioning (PPP): 
  // Solve ambiguity at given epoch
  // It returns true only when enough number (see "min_num_fixation") of 
  // narrow lane ambiguities are fixed
  Result solvePpp(const BackendId& epoch_id, 
             const std::vector<BackendId>& ambiguity_ids,
             const Eigen::MatrixXd& ambiguity_covariance,
             const GnssMeasurement& measurements);

  // For Real-Time Kinematic (RTK): 
  // Solve ambiguity for single differenced ambiguities (with double differenced measurements)
  Result solveRtk(const BackendId& epoch_id, 
              const std::vector<BackendId>& ambiguity_ids,
              const Eigen::MatrixXd& ambiguity_covariance,
              const std::pair<GnssMeasurement, GnssMeasurement>& measurements);

  // Set coordinate
  void setCoordinate(const GeoCoordinatePtr coordinate) { 
    coordinate_ = coordinate;
  }

  // Get options
  const AmbiguityResolutionOptions& getOptions() const { return options_; }

private:
  // Apply Between-Satellite-Difference (BSD) and lane combination for PPP
  void formSatellitePairPpp();

  // Apply Between-Satellite-Difference (BSD) and lane combination for RTK
  void formSatellitePairRtk();

  // Sort lanes to groups
  void groupingSatellitePair();

  // Sort lanes to groups for given band
  std::vector<std::vector<size_t>> 
  groupingSatellitePairForBand(const LaneType& type, const size_t min_num);

  // Solve ambiguities
  int solveLanes(const std::vector<size_t>& indexes, 
                 const double min_percentage_fixation,
                 const DelMethod method,
                 const bool use_rounding = false, 
                 const bool skip_full = false);

  // Try to solve ambiguity by all partial AR methods
  int trySolveLanes(const std::vector<size_t>& indexes, 
                 const double min_percentage_fixation,
                 const bool use_rounding = false);

  // Search match on the last pairs
  bool findMatch(LanePair& lane_pair,
                std::vector<LanePair>& matches,
                std::vector<double>& coefficients);

  // Check ambiguity stability and add fixation to graph. If one ambiguity was fixed to the 
  // same value for given epochs (see "min_consistant_fix_as_stable"), we consider it as 
  // stable. And then we constraint it on RTK estimator tightly.
  bool addStableFixationToGraph();

  // Erase all added ambiguity residual blocks 
  void eraseAmbiguityResidualBlocks(std::vector<LanePair>& lane_pairs);

  // Set graph parameter values
  void setGraphParameters(std::vector<Parameter>& parameters);

  // Compute pseudorange and phasernage total cost in current epoch
  double computeRangeCost(const BackendId& epoch_id);

  // research/vision-aided-ambiguity-resolution: robustified total cost of the joint
  // vision+IMU residuals (kReprojectionError + kIMUError) over the whole active graph,
  // excluding GNSS and the fixation constraints. Used by the opt-in joint post-fix
  // validation (see AmbiguityResolutionOptions::use_joint_cost_validation).
  double computeJointNonGnssCost();

  // research/vision-aided-ambiguity-resolution: theoretical integer-bootstrapping
  // success rate P_s of a float-ambiguity covariance, computed on the LAMBDA-
  // decorrelated conditional variances (LD factorization + integer Gauss reduction,
  // ported from the vendored RTKLIB lambda.c). Used by the opt-in
  // min_bootstrap_success_rate gate.
  static double bootstrapSuccessRate(const Eigen::MatrixXd& covariance);

  // Check if it is the first epoch
  bool isFirstEpoch() { return ambiguities_.size() < 2; }

  // Getters
  std::vector<Spec>& curAmbs() { return ambiguities_.back(); }
  std::vector<BsdPair>& curAmbPairs() { return ambiguity_pairs_.back(); }
  std::vector<LanePair>& curAmbLanePairs() { return ambiguity_lane_pairs_.back(); }
  std::vector<Spec>& lastAmbs() { 
    CHECK(ambiguities_.size() >= 2);
    return ambiguities_[ambiguities_.size() - 2]; 
  }
  std::vector<BsdPair>& lastAmbPairs() { 
    CHECK(ambiguity_pairs_.size() >= 2);
    return ambiguity_pairs_[ambiguity_pairs_.size() - 2]; 
  }
  std::vector<LanePair>& lastAmbLanePairs() { 
    CHECK(ambiguity_lane_pairs_.size() >= 2);
    return ambiguity_lane_pairs_[ambiguity_lane_pairs_.size() - 2]; 
  }

private:
  // Graph in RTK estimator class
  std::shared_ptr<Graph> graph_;

  // loss function
  std::shared_ptr<ceres::LossFunction> cauchy_loss_function_; 
  std::shared_ptr<ceres::LossFunction> huber_loss_function_; 

  // Ambiguity handles
  std::deque<std::vector<Spec>> ambiguities_;
  std::deque<std::vector<BsdPair>> ambiguity_pairs_;
  std::deque<std::vector<LanePair>> ambiguity_lane_pairs_;
  std::vector<Parameter> full_parameters_store_;
  std::vector<std::vector<size_t>> lane_groups_;

  // Covariances in minimal
  Eigen::MatrixXd ambiguity_covariance_;

  // Options
  AmbiguityResolutionOptions options_;

  // Coordinate handle
  GeoCoordinatePtr coordinate_;
};

}