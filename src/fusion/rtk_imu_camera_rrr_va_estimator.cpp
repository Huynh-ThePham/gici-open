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
#include <limits>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace gici {
namespace {

double eigenRoundoffTol(int dim, double scale)
{
  return std::numeric_limits<double>::epsilon() *
         static_cast<double>(std::max(1, dim)) * scale;
}

bool invertStrictSpd(const Eigen::MatrixXd& matrix, Eigen::MatrixXd& inverse)
{
  if (matrix.rows() == 0 || matrix.rows() != matrix.cols() || !matrix.allFinite()) {
    return false;
  }
  const Eigen::MatrixXd sym = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(sym);
  if (es.info() != Eigen::Success) return false;
  const double maxev = es.eigenvalues().maxCoeff();
  if (!(maxev > 0.0)) return false;
  const double tol = eigenRoundoffTol(static_cast<int>(matrix.rows()), maxev);
  if (es.eigenvalues().minCoeff() <= tol) return false;
  inverse = es.eigenvectors() * es.eigenvalues().cwiseInverse().asDiagonal() *
            es.eigenvectors().transpose();
  inverse = 0.5 * (inverse + inverse.transpose()).eval();
  return inverse.allFinite();
}

}  // namespace

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
  if (rrr_options_.ar_use_fast_marginal_covariance) {
    LOG(INFO) << "[vaar-fast-telemetry] version=1 enabled=1 benchmark="
              << (rrr_options_.benchmark_joint_ambiguity_covariance ? 1 : 0);
  }
}

RtkImuCameraRrrVaEstimator::~RtkImuCameraRrrVaEstimator()
{
  if (rrr_options_.ar_use_fast_marginal_covariance) {
    // Best-effort lifetime summary. The per-abstention event records below remain the
    // source of truth when an external SIGINT terminates file-mode runs without unwinding.
    LOG(INFO) << "[vaar-fast-summary] version=1"
              << " calls=" << fast_marginal_calls_
              << " ok=" << fast_marginal_successes_
              << " abstain=" << fast_marginal_abstentions_
              << " nuisance_not_psd=" << fast_marginal_nuisance_not_psd_;
  }
}

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
        parameter_block_ids, fast_cov, rrr_options_.ablate_reprojection_in_ar, &fail_reason);
    auto t1 = chrono::steady_clock::now();
    const double fast_ms = chrono::duration<double, std::milli>(t1 - t0).count();

    ++fast_marginal_calls_;
    if (ok) {
      ++fast_marginal_successes_;
    } else {
      ++fast_marginal_abstentions_;
      if (fail_reason == "nuisance_not_psd") {
        ++fast_marginal_nuisance_not_psd_;
      }
      // Never throttle this event: one line corresponds to exactly one fail-closed
      // fallback. The cumulative fields make omissions/duplicates detectable offline.
      LOG(INFO) << "[vaar-fast-abstain] t=" << std::fixed << std::setprecision(3)
                << state.timestamp
                << " n=" << num_ambiguities
                << " why=" << (fail_reason.empty() ? "unknown" : fail_reason)
                << " calls=" << fast_marginal_calls_
                << " abstain=" << fast_marginal_abstentions_
                << " nuisance_not_psd=" << fast_marginal_nuisance_not_psd_
                << " fast_ms=" << fast_ms;
    }

    if (rrr_options_.benchmark_joint_ambiguity_covariance) {
      // Equivalence + cost experiment: compare against the whole-problem ceres::Covariance
      // reference (the statistically-correct-but-slow path). This is the runtime validation
      // that the fast routine returns the same consistent covariance, on real graphs.
      auto t2 = chrono::steady_clock::now();
      Eigen::MatrixXd ceres_cov;
      const bool ceres_ok = graph_->computeCovariance(parameter_block_ids, ceres_cov);
      auto t3 = chrono::steady_clock::now();
      // rel_err = ||Q_fast - Q_ceres||_F / ||Q_ceres||_F (Frobenius, not trace).
      // Additional Loewner / directional diagnostics for paper-grade validation.
      double rel_err = -1.0;
      double tr_fast = -1.0, tr_ceres = -1.0;
      double max_diag_rel = -1.0;
      double min_eig_diff = std::numeric_limits<double>::quiet_NaN();
      double gen_eig_min = std::numeric_limits<double>::quiet_NaN();
      double gen_eig_max = std::numeric_limits<double>::quiet_NaN();
      int psd_diff = -1;  // 1 => Q_fast - Q_ceres PSD (proposed Loewner-larger)
      if (ok && ceres_ok && ceres_cov.rows() == fast_cov.rows() &&
          ceres_cov.allFinite() && fast_cov.allFinite() &&
          fast_cov.rows() == fast_cov.cols() && fast_cov.rows() > 0) {
        const Eigen::MatrixXd Qf = 0.5 * (fast_cov + fast_cov.transpose());
        const Eigen::MatrixXd Qc = 0.5 * (ceres_cov + ceres_cov.transpose());
        const double denom = Qc.norm();
        rel_err = denom > 0.0 ? (Qf - Qc).norm() / denom : -1.0;
        tr_fast = Qf.trace();
        tr_ceres = Qc.trace();

        max_diag_rel = 0.0;
        for (int i = 0; i < Qf.rows(); ++i) {
          const double cii = std::abs(Qc(i, i));
          if (cii > 0.0) {
            max_diag_rel = std::max(max_diag_rel, std::abs(Qf(i, i) - Qc(i, i)) / cii);
          } else if (Qf(i, i) != Qc(i, i)) {
            max_diag_rel = std::numeric_limits<double>::infinity();
          }
        }

        const Eigen::MatrixXd D = Qf - Qc;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_d(D);
        if (es_d.info() == Eigen::Success) {
          min_eig_diff = es_d.eigenvalues().minCoeff();
          const double scale = std::max(Qf.norm(), Qc.norm());
          psd_diff =
              (min_eig_diff >= -eigenRoundoffTol(static_cast<int>(Qf.rows()), scale)) ? 1 : 0;
        }

        // Generalized eigenvalues of pencil (Qf, Qc): Qc^{-1/2} Qf Qc^{-1/2}.
        // λ > 1 => proposed has larger variance in that direction.
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_c(Qc);
        if (es_c.info() == Eigen::Success) {
          const Eigen::VectorXd ev = es_c.eigenvalues();
          const double maxev = ev.maxCoeff();
          if (maxev > 0.0) {
            const double tol = eigenRoundoffTol(static_cast<int>(Qc.rows()), maxev);
            if (ev.minCoeff() > tol) {
              Eigen::VectorXd inv_sqrt = ev.cwiseSqrt().cwiseInverse();
              const Eigen::MatrixXd Qc_inv_sqrt =
                  es_c.eigenvectors() * inv_sqrt.asDiagonal() * es_c.eigenvectors().transpose();
              const Eigen::MatrixXd M =
                  Qc_inv_sqrt * Qf * Qc_inv_sqrt;
              Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_m(0.5 * (M + M.transpose()));
              if (es_m.info() == Eigen::Success) {
                gen_eig_min = es_m.eigenvalues().minCoeff();
                gen_eig_max = es_m.eigenvalues().maxCoeff();
              }
            }
          }
        }
      }
      LOG(INFO) << "[vaar-fast] t=" << std::fixed << std::setprecision(3)
        << state.timestamp << " n=" << num_ambiguities
        << " fast_ok=" << ok
        << " why=" << (ok ? "-" : fail_reason)
        << " ceres_ok=" << ceres_ok
        << " rel_err=" << std::setprecision(6) << rel_err
        << " tr_fast=" << tr_fast
        << " tr_ceres=" << tr_ceres
        << " max_diag_rel=" << max_diag_rel
        << " min_eig_diff=" << min_eig_diff
        << " psd_diff=" << psd_diff
        << " gen_eig_min=" << gen_eig_min
        << " gen_eig_max=" << gen_eig_max
        << " fast_ms=" << std::setprecision(3) << fast_ms
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
    const double roundoff_tol = eigenRoundoffTol(num_ambiguities, maxev);
    const double minev = es.eigenvalues().minCoeff();
    if (minev <= roundoff_tol) return false;

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
      Eigen::MatrixXd Hinv;
      if (!invertStrictSpd(H, Hinv)) return false;
      const Eigen::MatrixXd marg_cov =
        Hinv.topLeftCorner(num_ambiguities, num_ambiguities);
      return invertStrictSpd(marg_cov, info);
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
      shadow_cov.cols() != num_ambiguities ||
      !shadow_cov.allFinite()) return false;

  // Legacy convex-mix path (ablation only; not the default). Cov = gamma*local +
  // (1-gamma)*shadow -- NOT guaranteed <= shadow, hence the Medium fixed-rate
  // regression. Kept for the paper's fusion-method comparison.
  if (!rrr_options_.ar_use_delta_information) {
    Eigen::MatrixXd H_with_inv;
    if (!invertStrictSpd(H_with, H_with_inv)) return false;
    const Eigen::MatrixXd local_cov =
        H_with_inv.topLeftCorner(num_ambiguities, num_ambiguities);
    Eigen::MatrixXd unused_shadow_info;
    if (!invertStrictSpd(shadow_cov, unused_shadow_info)) return false;
    const double gamma = rrr_options_.vision_ar_covariance_mix;
    if (!(gamma >= 0.0 && gamma <= 1.0)) return false;
    covariance = gamma * local_cov + (1.0 - gamma) * shadow_cov;
    covariance = 0.5 * (covariance + covariance.transpose()).eval();
    return covariance.allFinite();
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
    // A_with - A_without is PSD in exact arithmetic: adding reprojection information to
    // the nuisance block can only increase the Schur-complement ambiguity information.
    // A material negative eigenvalue means the local approximation is numerically
    // unresolved, so abstain instead of projecting it to a more confident PSD matrix.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> esd(delta);
    if (esd.info() != Eigen::Success) return false;
    const double scale = esd.eigenvalues().cwiseAbs().maxCoeff();
    if (esd.eigenvalues().minCoeff() <
        -eigenRoundoffTol(static_cast<int>(delta.rows()), scale)) return false;
  }

  Eigen::MatrixXd I_shadow;
  if (!invertStrictSpd(shadow_cov, I_shadow)) return false;

  const Eigen::MatrixXd I_final = I_shadow + delta;
  if (!invertStrictSpd(I_final, covariance)) return false;

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
