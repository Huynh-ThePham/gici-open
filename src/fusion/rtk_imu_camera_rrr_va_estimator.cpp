/**
* @Function: RTK/IMU/Camera RRR estimator with genuinely vision-aided ambiguity
*             resolution (research/vision-aided-ambiguity-resolution).
*
* See include/gici/fusion/rtk_imu_camera_rrr_va_estimator.h and
* research/VISION_AIDED_AR.md.
**/
#include "gici/fusion/rtk_imu_camera_rrr_va_estimator.h"

#include <unordered_set>
#include <string>
#include <algorithm>
#include <chrono>
#include <iomanip>

namespace gici {

RtkImuCameraRrrVaEstimator::RtkImuCameraRrrVaEstimator(
               const RtkImuCameraRrrEstimatorOptions& options,
               const GnssImuInitializerOptions& init_options,
               const RtkEstimatorOptions rtk_options,
               const GnssEstimatorBaseOptions& gnss_base_options,
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options,
               const VisualEstimatorBaseOptions& visual_base_options,
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options) :
  // EstimatorBase is a virtual base (diamond via GnssEstimatorBase/VisualEstimatorBase/
  // ImuEstimatorBase), so the most-derived class must initialize it directly; the base
  // RtkImuCameraRrrEstimator's own EstimatorBase(base_options) is then ignored (same args).
  EstimatorBase(base_options),
  RtkImuCameraRrrEstimator(options, init_options, rtk_options, gnss_base_options,
    gnss_loose_base_options, visual_base_options, imu_base_options, base_options,
    ambiguity_options)
{
  type_ = EstimatorType::RtkImuCameraRrrVa;
  // This estimator type IS the genuinely-vision-aided AR estimator: force the
  // vision-aided AR path on so estimate() invokes our overridden covariance method,
  // regardless of the config flag value.
  rrr_options_.use_vision_aided_ambiguity_resolution = true;
}

RtkImuCameraRrrVaEstimator::~RtkImuCameraRrrVaEstimator()
{}

// Genuinely vision-aided ambiguity covariance. The default path asks the real joint
// graph for the covariance of the current ambiguity blocks only. Ceres computes that
// marginal covariance by eliminating every other active parameter block in the same
// problem (poses, speed/bias, landmarks, extrinsics, marginalization priors, etc.),
// which is the consistent full Schur-complement treatment. The older bounded local
// delta-information path remains below only for ablation when explicitly requested.
bool RtkImuCameraRrrVaEstimator::estimateVisionAidedAmbiguityCovariance(
  const State& state, Eigen::MatrixXd& covariance)
{
  const int num_ambiguities = static_cast<int>(curAmbiguityState().ids.size());
  if (num_ambiguities == 0) return false;

  std::vector<uint64_t> parameter_block_ids;

  // Ambiguities (must all exist in the graph; ordering matches solveRtk()).
  for (auto id : curAmbiguityState().ids) {
    const uint64_t key = id.asInteger();
    if (!graph_->parameterBlockExists(key)) return false;
    parameter_block_ids.push_back(key);
  }
  if (static_cast<int>(parameter_block_ids.size()) != num_ambiguities) return false;

  // Fast exact-in-window marginal covariance (default VA path). Computes the SAME quantity
  // as the exact ceres path below -- Q_aa = [H_active^-1]_aa, marginalizing every other
  // active state including the marginalization prior -- but by structure-exploiting sparse
  // Schur elimination, so it is usable at every AR epoch in real time. Returns false (caller
  // falls back to the plain shadow covariance) when the reduced ambiguity information is not
  // positive-definite; it never fabricates confidence.
  if (rrr_options_.ar_use_fast_marginal_covariance) {
    namespace chrono = std::chrono;
    auto t0 = chrono::steady_clock::now();
    Eigen::MatrixXd fast_cov;
    std::string fail_reason;
    const bool ok = graph_->getMarginalAmbiguityCovariance(
        parameter_block_ids, fast_cov, &fail_reason);
    auto t1 = chrono::steady_clock::now();

    if (rrr_options_.benchmark_joint_ambiguity_covariance) {
      // Equivalence + cost experiment: compare against the whole-problem ceres::Covariance
      // reference (the statistically-correct-but-slow path). This is the runtime validation
      // that the fast routine returns the same consistent covariance, on real graphs.
      auto t2 = chrono::steady_clock::now();
      Eigen::MatrixXd ceres_cov;
      const bool ceres_ok = graph_->computeCovariance(parameter_block_ids, ceres_cov);
      auto t3 = chrono::steady_clock::now();
      double rel_err = -1.0;
      double tr_fast = -1.0, tr_ceres = -1.0;
      if (ok && ceres_ok && ceres_cov.rows() == fast_cov.rows() &&
          ceres_cov.allFinite() && fast_cov.allFinite()) {
        const double denom = ceres_cov.norm();
        rel_err = denom > 0.0 ? (fast_cov - ceres_cov).norm() / denom : -1.0;
        tr_fast = fast_cov.trace();
        tr_ceres = ceres_cov.trace();
      }
      LOG(INFO) << "[vaar-fast] t=" << std::fixed << std::setprecision(3)
        << state.timestamp << " n=" << num_ambiguities
        << " fast_ok=" << ok
        << " why=" << (ok ? "-" : fail_reason)
        << " ceres_ok=" << ceres_ok
        << " rel_err=" << std::setprecision(6) << rel_err
        << " tr_fast=" << tr_fast
        << " tr_ceres=" << tr_ceres
        << " fast_ms=" << std::setprecision(3)
        << chrono::duration<double, std::milli>(t1 - t0).count()
        << " ceres_ms=" << chrono::duration<double, std::milli>(t3 - t2).count()
        << " graph_pb=" << graph_->parameters().size()
        << " graph_rb=" << graph_->residuals().size();
    }

    if (!ok) return false;
    covariance = fast_cov;
    return true;
  }

  if (rrr_options_.ar_use_exact_joint_covariance) {
    namespace chrono = std::chrono;
    auto t0 = chrono::steady_clock::now();
    Eigen::MatrixXd exact_covariance;
    if (!graph_->computeCovariance(parameter_block_ids, exact_covariance)) {
      return false;
    }
    auto t1 = chrono::steady_clock::now();

    if (exact_covariance.rows() != num_ambiguities ||
        exact_covariance.cols() != num_ambiguities ||
        !exact_covariance.allFinite()) {
      return false;
    }
    exact_covariance = 0.5 * (exact_covariance + exact_covariance.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(exact_covariance);
    if (es.info() != Eigen::Success) return false;
    const double maxev = es.eigenvalues().maxCoeff();
    if (!(maxev > 0.0)) return false;
    const double roundoff_tol = 1.0e-10 * std::max(1.0, maxev);
    const double minev = es.eigenvalues().minCoeff();
    if (minev < -roundoff_tol) return false;
    if (minev <= 0.0) {
      exact_covariance = es.eigenvectors()
        * es.eigenvalues().cwiseMax(roundoff_tol).asDiagonal()
        * es.eigenvectors().transpose();
    }

    covariance = exact_covariance;
    if (rrr_options_.benchmark_joint_ambiguity_covariance) {
      const double exact_ms =
        chrono::duration<double, std::milli>(t1 - t0).count();
      LOG(INFO) << "[vaar-exact] t=" << std::fixed << std::setprecision(3)
        << state.timestamp
        << " n=" << num_ambiguities
        << " exact_ms=" << std::setprecision(3) << exact_ms
        << " tr_cov=" << std::setprecision(6) << covariance.trace()
        << " min_eig=" << minev
        << " graph_num_parameter_blocks=" << graph_->parameters().size()
        << " graph_num_residual_blocks=" << graph_->residuals().size();
    }
    return true;
  }

  std::unordered_set<uint64_t> added(parameter_block_ids.begin(),
                                     parameter_block_ids.end());

  // Current GNSS pose + its speed-and-bias (as in the base approach).
  const uint64_t gpose_key = state.id_in_graph.asInteger();
  if (!graph_->parameterBlockExists(gpose_key)) return false;
  parameter_block_ids.push_back(gpose_key);
  added.insert(gpose_key);
  const uint64_t gsb_key =
    changeIdType(state.id_in_graph, IdType::ImuStates).asInteger();
  if (graph_->parameterBlockExists(gsb_key) && added.insert(gsb_key).second) {
    parameter_block_ids.push_back(gsb_key);
  }

  // Extend with the chain back to the most recent camera keyframe (cPose) so that
  // reprojection residuals attached to that cPose enter the joint information (kept
  // connected to the current pose through the intervening IMU pre-integration factors).
  // The search spans the whole current optimization window (states_) rather than an
  // arbitrary fixed look-back: the window itself (bounded by max_keyframes) bounds how
  // stale the included keyframe can be, so no magic-number cap is needed. If no cPose
  // is in the window, this degrades gracefully to the current-pose-only local matrix.
  int num_camera_keyframes = 0;
  const int start = static_cast<int>(latest_state_index_);
  if (start >= 0 && start < static_cast<int>(states_.size())) {
    int cpose_index = -1;
    for (int i = start; i >= 0; --i) {
      if (states_[i].id.type() == IdType::cPose) { cpose_index = i; break; }
    }
    if (cpose_index >= 0) {
      for (int i = start - 1; i >= cpose_index; --i) {
        const uint64_t pose_key = states_[i].id_in_graph.asInteger();
        if (!graph_->parameterBlockExists(pose_key)) continue;
        if (added.insert(pose_key).second) parameter_block_ids.push_back(pose_key);
        const uint64_t sb_key =
          changeIdType(states_[i].id_in_graph, IdType::ImuStates).asInteger();
        if (graph_->parameterBlockExists(sb_key) && added.insert(sb_key).second) {
          parameter_block_ids.push_back(sb_key);
        }
        if (states_[i].id.type() == IdType::cPose) num_camera_keyframes++;
      }
    }
  }

  // Helper: marginal ambiguity information from a local joint information matrix H over
  // [ambiguities ; pose/keyframe chain]. Integrates out the pose block via the full
  // inverse top-left block (Schur complement), then re-inverts to information form.
  // Returns false if H is not positive definite (rank-deficient local geometry) so the
  // caller can fall back to the plain shadow covariance -- never fabricating confidence.
  auto marginalAmbiguityInformation =
    [num_ambiguities](const Eigen::MatrixXd& H, Eigen::MatrixXd& info) -> bool {
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H);
      if (es.info() != Eigen::Success) return false;
      const double maxev = es.eigenvalues().maxCoeff();
      if (!(maxev > 0.0)) return false;
      if (es.eigenvalues().minCoeff() < 1e-9 * std::max(1.0, maxev)) return false;
      const Eigen::MatrixXd Hinv = es.eigenvectors()
        * es.eigenvalues().cwiseInverse().asDiagonal()
        * es.eigenvectors().transpose();
      const Eigen::MatrixXd marg_cov =
        Hinv.topLeftCorner(num_ambiguities, num_ambiguities);
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esc(marg_cov);
      if (esc.info() != Eigen::Success ||
          esc.eigenvalues().minCoeff() < 1e-12) return false;
      info = esc.eigenvectors()
        * esc.eigenvalues().cwiseInverse().asDiagonal()
        * esc.eigenvectors().transpose();
      return true;
    };

  // Two local informations over the SAME requested blocks: WITH and WITHOUT the camera
  // reprojection residuals. Reprojection attaches only to camera-pose blocks, so the
  // ambiguity and ambiguity-pose blocks are identical between the two matrices -- only
  // the pose block differs, by a PSD reprojection term. The ablation flag makes the
  // "with" call also drop reprojection, collapsing the vision increment to zero (i.e.
  // reproducing the baseline shadow covariance) to isolate the genuine visual gain.
  Eigen::MatrixXd H_with, H_without;
  if (!graph_->getLocalCrossInformation(parameter_block_ids, H_with,
        rrr_options_.ablate_reprojection_in_ar)) return false;
  if (!graph_->getLocalCrossInformation(parameter_block_ids, H_without, true)) return false;

  // Shadow (GNSS-only) covariance. Already carries the current-epoch GNSS and the
  // absolute/2-epoch position uncertainty (large during/after an outage).
  Eigen::MatrixXd shadow_cov;
  if (!estimateAmbiguityCovariance(state, shadow_cov)) return false;
  if (shadow_cov.rows() != num_ambiguities ||
      shadow_cov.cols() != num_ambiguities) return false;

  // Legacy convex-mix path (ablation only; not the default). Cov = gamma*local +
  // (1-gamma)*shadow -- NOT guaranteed <= shadow, hence the Medium fixed-rate
  // regression. Kept for the paper's fusion-method comparison.
  if (!rrr_options_.ar_use_delta_information) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H_with);
    if (es.info() != Eigen::Success || es.eigenvalues().minCoeff() < 1e-9) return false;
    const Eigen::MatrixXd local_cov = (es.eigenvectors()
      * es.eigenvalues().cwiseInverse().asDiagonal()
      * es.eigenvectors().transpose()).topLeftCorner(num_ambiguities, num_ambiguities);
    const double gamma = rrr_options_.vision_ar_covariance_mix;
    covariance = (gamma >= 1.0) ? local_cov
                                : gamma * local_cov + (1.0 - gamma) * shadow_cov;
    return true;
  }

  // Delta-information fusion (default): I_final = I_shadow + (A_with - A_without).
  // This isolates the reprojection-driven local information increment and avoids the
  // earlier explicit double-count of the current GNSS residual. It is still approximate:
  // residual-connected blocks that are not requested above are conditioned at their
  // current values, not marginalized. Empirically this can be overconfident, so the
  // resulting covariance is experimental and carries no regression-safety guarantee.
  Eigen::MatrixXd A_with, A_without;
  if (!marginalAmbiguityInformation(H_with, A_with)) return false;
  if (!marginalAmbiguityInformation(H_without, A_without)) return false;

  Eigen::MatrixXd delta = A_with - A_without;
  delta = 0.5 * (delta + delta.transpose());
  {
    // Clamp to PSD (guards tiny negative eigenvalues from finite-precision arithmetic).
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esd(delta);
    if (esd.info() != Eigen::Success) return false;
    delta = esd.eigenvectors()
      * esd.eigenvalues().cwiseMax(0.0).asDiagonal()
      * esd.eigenvectors().transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> ess(shadow_cov);
  if (ess.info() != Eigen::Success || ess.eigenvalues().minCoeff() < 1e-12) return false;
  const Eigen::MatrixXd I_shadow = ess.eigenvectors()
    * ess.eigenvalues().cwiseInverse().asDiagonal()
    * ess.eigenvectors().transpose();

  const Eigen::MatrixXd I_final = I_shadow + delta;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esf(I_final);
  if (esf.info() != Eigen::Success || esf.eigenvalues().minCoeff() < 1e-12) return false;
  covariance = esf.eigenvectors()
    * esf.eigenvalues().cwiseInverse().asDiagonal()
    * esf.eigenvectors().transpose();

  // Diagnostic (off by default): how many camera keyframes entered, the covariance
  // trace change, and the size of the experimental vision information increment.
  if (rrr_options_.benchmark_joint_ambiguity_covariance) {
    const double tr_shadow = shadow_cov.trace();
    const double tr_fused = covariance.trace();
    LOG(INFO) << "[vaar-va] t=" << std::fixed << std::setprecision(3) << state.timestamp
      << " n=" << num_ambiguities
      << " kf=" << num_camera_keyframes
      << " dim=" << H_with.rows() << std::setprecision(5)
      << " tr_shadow=" << tr_shadow
      << " tr_fused=" << tr_fused
      << " fused/shadow=" << (tr_shadow > 0 ? tr_fused / tr_shadow : -1.0)
      << " tr_dInfo=" << delta.trace();
  }
  return true;
}

}  // namespace gici
