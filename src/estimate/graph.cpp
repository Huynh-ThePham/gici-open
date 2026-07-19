/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 8, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *    Modified: Zurich Eye
 *    Modified: Cheng Chi
 *********************************************************************************/

/**
 * @file Graph.cpp
 * @brief Source file for the Graph class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include "gici/estimate/graph.h"

#include <ceres/ordered_groups.h>
#include <unordered_set>
#include <limits>
#include <vector>

#include "gici/estimate/homogeneous_point_parameter_block.h"
#include "gici/estimate/marginalization_error.h"
#include "gici/estimate/estimator_types.h"

namespace gici {

// Constructor.
Graph::Graph()
    : residual_counter_(0)
{
  ceres::Problem::Options problemOptions;
  problemOptions.local_parameterization_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.loss_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  //problemOptions.enable_fast_parameter_block_removal = true;
  problem_.reset(new ceres::Problem(problemOptions));
  //options.linear_solver_ordering = new ceres::ParameterBlockOrdering;
}

// Check whether a certain parameter block is part of the graph.
bool Graph::parameterBlockExists(uint64_t parameter_block_id) const
{
  if (id_to_parameter_block_map_.find(parameter_block_id)
      == id_to_parameter_block_map_.end())
  {
    return false;
  }
  return true;
}

// Log information on a parameter block.
void Graph::printParameterBlockInfo(uint64_t parameter_block_id) const
{
  ResidualBlockCollection residual_collection = residuals(parameter_block_id);
  LOG(INFO) << "parameter info" << std::endl << "----------------------------"
            << std::endl << " - block Id: " << parameter_block_id << std::endl
            << " - type: " << parameterBlockPtr(parameter_block_id)->typeInfo()
            << std::endl << " - residuals (" << residual_collection.size()
            << "):";
  for (size_t i = 0; i < residual_collection.size(); ++i) {
    LOG(INFO)
        << "   - id: "
        << residual_collection.at(i).residual_block_id
        << std::endl
        << "   - type: "
        << kErrorToStr.at(errorInterfacePtr(residual_collection.at(i).residual_block_id)->typeInfo());
  }
  LOG(INFO) << "============================";
}

// Log information on a residual block.
void Graph::printResidualBlockInfo(
    ceres::ResidualBlockId residual_block_id) const
{
  LOG(INFO) << "   - id: " << residual_block_id << std::endl << "   - type: "
            << kErrorToStr.at(errorInterfacePtr(residual_block_id)->typeInfo());
}

// Obtain the Hessian block for a specific parameter block.
void Graph::getLhs(uint64_t parameter_block_id, Eigen::MatrixXd& H)
{
  CHECK(parameterBlockExists(parameter_block_id))
      << "parameter block not in graph.";
  ResidualBlockCollection res = residuals(parameter_block_id);
  H.setZero();
  for (size_t i = 0; i < res.size(); ++i)
  {

    // parameters:
    ParameterBlockCollection pars = parameters(res[i].residual_block_id);

    double** parameters_raw = new double*[pars.size()];
    Eigen::VectorXd residuals_eigen(res[i].error_interface_ptr->residualDim());
    double* residuals_raw = residuals_eigen.data();

    double** jacobians_raw = new double*[pars.size()];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > > jacobiansEigen(pars.size());

    double** jacobians_minimal_raw = new double*[pars.size()];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > > jacobians_minimal_eigen(pars.size());

    int J = -1;
    for (size_t j = 0; j < pars.size(); ++j)
    {
      // determine which is the relevant block
      if (pars[j].second->id() == parameter_block_id)
        J = j;
      parameters_raw[j] = pars[j].second->parameters();
      jacobiansEigen[j].resize(res[i].error_interface_ptr->residualDim(),
                               pars[j].second->dimension());
      jacobians_raw[j] = jacobiansEigen[j].data();
      jacobians_minimal_eigen[j].resize(res[i].error_interface_ptr->residualDim(),
                                      pars[j].second->minimalDimension());
      jacobians_minimal_raw[j] = jacobians_minimal_eigen[j].data();
    }

    // evaluate residual block
    res[i].error_interface_ptr->EvaluateWithMinimalJacobians(parameters_raw,
                                                           residuals_raw,
                                                           jacobians_raw,
                                                           jacobians_minimal_raw);

    // get block
    H += jacobians_minimal_eigen[J].transpose() * jacobians_minimal_eigen[J];

    // cleanup
    delete[] parameters_raw;
    delete[] jacobians_raw;
    delete[] jacobians_minimal_raw;
  }
}

// research/vision-aided-ambiguity-resolution: local information matrix (with cross
// terms) over a small set of parameter blocks -- see graph.h for the full rationale.
bool Graph::getLocalCrossInformation(
    const std::vector<uint64_t>& parameter_block_ids, Eigen::MatrixXd& information,
    bool exclude_reprojection)
{
  std::vector<size_t> offsets(parameter_block_ids.size());
  std::vector<size_t> sizes(parameter_block_ids.size());
  size_t total_size = 0;
  std::unordered_map<uint64_t, size_t> id_to_index;
  for (size_t k = 0; k < parameter_block_ids.size(); k++) {
    if (!parameterBlockExists(parameter_block_ids[k])) return false;
    auto block = parameterBlockPtr(parameter_block_ids[k]);
    offsets[k] = total_size;
    sizes[k] = block->minimalDimension();
    total_size += sizes[k];
    id_to_index[parameter_block_ids[k]] = k;
  }
  information.setZero(total_size, total_size);

  // Union of residual blocks touching any requested parameter block, deduplicated
  // (a residual can be reached via more than one of the requested blocks).
  //
  // The marginalization prior is deliberately excluded: (1) it summarizes already-
  // marginalized PAST states, so folding it into a "current-epoch local information"
  // matrix conflates accumulated history with the current measurement -- not what this
  // routine is meant to capture; and (2) its Jacobian is dynamically sized and its
  // EvaluateWithMinimalJacobians writes e0_.rows()-row blocks whose interaction with
  // this routine's per-residual raw-buffer evaluation is fragile. Skipping it keeps the
  // local information a clean sum of current-epoch measurement residuals. (Approach-A
  // callers that request only {ambiguities, current pose, speed-bias} never reach the
  // prior anyway, since it attaches to the oldest window-boundary states, so this does
  // not change their results.)
  std::unordered_set<ceres::ResidualBlockId> seen_residuals;
  ResidualBlockCollection touched_residuals;
  for (auto id : parameter_block_ids) {
    for (auto& r : residuals(id)) {
      const ErrorType type = r.error_interface_ptr->typeInfo();
      if (type == ErrorType::kMarginalizationError) continue;
      if (exclude_reprojection && type == ErrorType::kReprojectionError) continue;
      if (seen_residuals.insert(r.residual_block_id).second) {
        touched_residuals.push_back(r);
      }
    }
  }

  for (auto& res : touched_residuals) {
    ParameterBlockCollection pars = parameters(res.residual_block_id);

    double** parameters_raw = new double*[pars.size()];
    Eigen::VectorXd residuals_eigen(res.error_interface_ptr->residualDim());
    double* residuals_raw = residuals_eigen.data();

    double** jacobians_raw = new double*[pars.size()];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > > jacobiansEigen(pars.size());

    double** jacobians_minimal_raw = new double*[pars.size()];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > > jacobians_minimal_eigen(pars.size());

    // Which requested-block index (if any) each of this residual's touched
    // parameters corresponds to; parameters not in parameter_block_ids (e.g. a
    // previous pose, or visual landmarks) get -1 and are evaluated (fixed at their
    // current value) but not accumulated -- this is the "condition on current
    // estimate" approximation described in graph.h.
    std::vector<int> local_index(pars.size(), -1);
    for (size_t j = 0; j < pars.size(); ++j) {
      auto it = id_to_index.find(pars[j].second->id());
      if (it != id_to_index.end()) local_index[j] = static_cast<int>(it->second);
      parameters_raw[j] = pars[j].second->parameters();
      jacobiansEigen[j].resize(res.error_interface_ptr->residualDim(),
                               pars[j].second->dimension());
      jacobians_raw[j] = jacobiansEigen[j].data();
      jacobians_minimal_eigen[j].resize(res.error_interface_ptr->residualDim(),
                                      pars[j].second->minimalDimension());
      jacobians_minimal_raw[j] = jacobians_minimal_eigen[j].data();
    }

    res.error_interface_ptr->EvaluateWithMinimalJacobians(parameters_raw,
                                                           residuals_raw,
                                                           jacobians_raw,
                                                           jacobians_minimal_raw);

    for (size_t a = 0; a < pars.size(); ++a) {
      if (local_index[a] < 0) continue;
      const size_t row_off = offsets[local_index[a]];
      const size_t row_size = sizes[local_index[a]];
      for (size_t b = 0; b < pars.size(); ++b) {
        if (local_index[b] < 0) continue;
        const size_t col_off = offsets[local_index[b]];
        const size_t col_size = sizes[local_index[b]];
        information.block(row_off, col_off, row_size, col_size) +=
          jacobians_minimal_eigen[a].transpose() * jacobians_minimal_eigen[b];
      }
    }

    delete[] parameters_raw;
    delete[] jacobians_raw;
    delete[] jacobians_minimal_raw;
  }

  return true;
}

// research/vision-aided-ambiguity-resolution: exact-in-window marginal ambiguity
// covariance via structure-exploiting Schur elimination. See graph.h for the contract.
//
// We assemble the loss-corrected Gauss-Newton information of the ACTIVE graph in minimal
// coordinates, partitioned as
//     H = [ H_aa  H_an   0   ]     a = ambiguities (given order)
//         [ H_na  H_nn  H_nl ]     n = dense nuisance (poses, speed/bias, clocks, ...)
//         [  0    H_ln  H_ll ]     l = visual landmarks (H_ll block-diagonal 3x3;
//                                      H_al = 0 structurally)
// including the marginalization prior, then Schur-eliminate l block-by-block (rank-
// truncated per landmark) and n densely, returning Q_aa = S^{-1} with
//     S = H_aa - H_an (H_nn - H_nl H_ll^+ H_ln)^{-1} H_na.
// Because Schur complements compose and H_al = 0, this equals [H_active^{-1}]_aa up to
// the linearization -- the same quantity computeCovariance() (ceres::Covariance) returns
// -- at a cost bounded by the sliding window, not the trajectory (real-time). The
// staged elimination also isolates weak-parallax landmarks (rank-truncated locally) so
// they cannot poison the conditioning of the global elimination, and every path is
// gated so the routine abstains (returns false) when normal-equations arithmetic cannot
// numerically resolve the marginal -- it never fabricates confidence.
bool Graph::getMarginalAmbiguityCovariance(
    const std::vector<uint64_t>& ambiguity_ids, Eigen::MatrixXd& Q_aa,
    std::string* fail_reason)
{
  auto fail = [&](const char* reason) -> bool {
    if (fail_reason) *fail_reason = reason;
    return false;
  };
  const int num_amb = static_cast<int>(ambiguity_ids.size());
  if (num_amb == 0) return fail("no_ambiguities");

  const double eps = std::numeric_limits<double>::epsilon();

  // ---- Pass 0: locate the marginalization prior (if any) and read its information +
  // connected block ids up front. Blocks referenced by the prior must not be classified
  // as independently-eliminable landmark blocks below: the prior can couple them to any
  // other state, which would break the landmark-block independence stage 1 relies on.
  const ResidualBlockCollection all_residuals = residuals();
  std::vector<uint64_t> prior_ids;
  std::vector<size_t> prior_off, prior_dim;
  Eigen::MatrixXd prior_Lambda;
  bool have_prior = false;
  for (const auto& res : all_residuals) {
    if (res.error_interface_ptr->typeInfo() != ErrorType::kMarginalizationError) continue;
    if (have_prior) return fail("multiple_priors");
    auto prior = std::dynamic_pointer_cast<MarginalizationError>(res.error_interface_ptr);
    if (!prior) return fail("prior_cast");
    if (!prior->marginalizationInformation(prior_ids, prior_off, prior_dim, prior_Lambda)) {
      return fail("prior_info_unavailable");
    }
    have_prior = true;
  }
  std::unordered_set<uint64_t> prior_id_set(prior_ids.begin(), prior_ids.end());

  // ---- Classification. Minimal-coordinate partitions:
  //   A: the requested ambiguities (scalar), in the caller's order;
  //   L: visual landmarks (homogeneous points: ambient dim 4, minimal dim 3) NOT touched
  //      by the prior -- eliminated first, block-by-block, with per-block rank truncation
  //      (this is what keeps weak-parallax landmarks from poisoning the conditioning of
  //      the remaining dense elimination);
  //   N: every other active block (poses, speed/bias, clocks/frequencies, extrinsics, and
  //      any prior-touched landmark) -- eliminated second, densely.
  // Fixed/constant blocks are excluded entirely (they contribute no covariance dimension,
  // matching ceres, which conditions on constant blocks).
  enum class Part : uint8_t { A, N, L };
  struct Slot { Part part; int offset; int dim; };  // offset within its own partition
  std::unordered_map<uint64_t, Slot> slot;
  slot.reserve(2048);
  for (int k = 0; k < num_amb; ++k) {
    const uint64_t id = ambiguity_ids[k];
    auto it = id_to_parameter_block_map_.find(id);
    if (it == id_to_parameter_block_map_.end()) return fail("ambiguity_absent");
    if (it->second->fixed()) return fail("ambiguity_fixed");
    if (it->second->minimalDimension() != 1) return fail("ambiguity_not_scalar");
    if (!slot.emplace(id, Slot{Part::A, k, 1}).second) return fail("duplicate_ambiguity");
  }
  int n_dim = 0;
  int num_landmarks = 0;
  auto slot_of = [&](const std::shared_ptr<ParameterBlock>& pb) -> const Slot* {
    const uint64_t id = pb->id();
    auto it = slot.find(id);
    if (it != slot.end()) return &it->second;
    if (pb->fixed()) return nullptr;
    const int md = static_cast<int>(pb->minimalDimension());
    if (md <= 0) return nullptr;
    const bool is_landmark = (pb->dimension() == 4 && md == 3 &&
                              prior_id_set.find(id) == prior_id_set.end());
    if (is_landmark) {
      auto r = slot.emplace(id, Slot{Part::L, num_landmarks, 3});
      num_landmarks++;
      return &r.first->second;
    }
    auto r = slot.emplace(id, Slot{Part::N, n_dim, md});
    n_dim += md;
    return &r.first->second;
  };

  // ---- Pass 1: discover all active blocks touched by any residual, fixing n_dim and the
  // landmark count before sizing containers. ----
  for (const auto& res : all_residuals) {
    const ParameterBlockCollection pars = parameters(res.residual_block_id);
    for (const auto& p : pars) slot_of(p.second);
  }

  // ---- Containers. Dense H_aa / H_an / H_nn (window-bounded sizes); per-landmark 3x3
  // information plus its few coupling strips to N-blocks (a landmark is observed by a
  // handful of keyframes, so the strip list stays tiny). ----
  Eigen::MatrixXd H_aa = Eigen::MatrixXd::Zero(num_amb, num_amb);
  Eigen::MatrixXd H_an = Eigen::MatrixXd::Zero(num_amb, std::max(n_dim, 1));
  Eigen::MatrixXd H_nn = Eigen::MatrixXd::Zero(std::max(n_dim, 1), std::max(n_dim, 1));
  struct LandmarkSys {
    Eigen::Matrix3d Hll = Eigen::Matrix3d::Zero();
    // (n_offset, dim_n x 3 coupling block); linear find is fine at this size.
    std::vector<std::pair<int, Eigen::MatrixXd>> strips;
    Eigen::MatrixXd& strip(int off, int dim) {
      for (auto& s : strips) {
        if (s.first == off) return s.second;
      }
      strips.emplace_back(off, Eigen::MatrixXd::Zero(dim, 3));
      return strips.back().second;
    }
  };
  std::vector<LandmarkSys> lms(num_landmarks);

  // Scatter one (row-block, col-block) contribution C into the partitioned containers.
  // Both orderings of every pair are visited, so one-sided cases can be skipped. The
  // partition structure guarantees no ambiguity<->landmark coupling (no residual connects
  // them, and prior-touched landmarks were promoted to N); if it is ever violated the
  // routine aborts cleanly rather than mis-assembling.
  bool structure_ok = true;
  auto scatter = [&](const Slot* sa, const Slot* sb, const Eigen::MatrixXd& C) {
    if (sa->part == Part::A) {
      if (sb->part == Part::A) {
        H_aa.block(sa->offset, sb->offset, sa->dim, sb->dim) += C;
      } else if (sb->part == Part::N) {
        H_an.block(sa->offset, sb->offset, sa->dim, sb->dim) += C;
      } else {
        structure_ok = false;  // A-L coupling: partition assumption violated
      }
    } else if (sa->part == Part::N) {
      if (sb->part == Part::N) {
        H_nn.block(sa->offset, sb->offset, sa->dim, sb->dim) += C;
      } else if (sb->part == Part::L) {
        lms[sb->offset].strip(sa->offset, sa->dim) += C;
      }
      // (N,A): transpose of (A,N), added when the swapped pair is visited.
    } else {  // sa in L
      if (sb->part == Part::L) {
        if (sa->offset != sb->offset) { structure_ok = false; return; }
        lms[sa->offset].Hll += C;
      } else if (sb->part == Part::A) {
        structure_ok = false;  // L-A coupling: partition assumption violated
      }
      // (L,N): transpose of (N,L), added when the swapped pair is visited.
    }
  };

  // ---- Pass 2: assemble H. ----
  for (const auto& res : all_residuals) {
    const ErrorType type = res.error_interface_ptr->typeInfo();

    // Marginalization prior: scatter its information Lambda = J_^T J_ (pre-fetched in
    // pass 0) directly -- never route the dynamically-sized prior through the generic
    // evaluation buffers.
    if (type == ErrorType::kMarginalizationError) {
      const int np = static_cast<int>(prior_ids.size());
      // Resolve each prior block to its active slot (skip fixed/absent).
      std::vector<const Slot*> pslot(np, nullptr);
      for (int i = 0; i < np; ++i) {
        auto it = slot.find(prior_ids[i]);
        if (it == slot.end()) continue;                 // fixed now / not active -> skip
        if (static_cast<int>(prior_dim[i]) != it->second.dim) return fail("prior_layout_mismatch");
        pslot[i] = &it->second;
      }
      for (int i = 0; i < np; ++i) {
        if (!pslot[i]) continue;
        for (int j = 0; j < np; ++j) {
          if (!pslot[j]) continue;
          scatter(pslot[i], pslot[j],
                  prior_Lambda.block(prior_off[i], prior_off[j],
                                     prior_dim[i], prior_dim[j]));
        }
      }
      continue;
    }

    // Ordinary residual: evaluate minimal Jacobians (same buffer pattern as
    // getLocalCrossInformation), then apply the loss-function correction so the assembled
    // information matches the robustified Hessian ceres uses.
    const ParameterBlockCollection pars = parameters(res.residual_block_id);
    const int rdim = static_cast<int>(res.error_interface_ptr->residualDim());
    Eigen::VectorXd residuals_eigen(rdim);

    std::vector<double*> parameters_raw(pars.size());
    std::vector<double*> jacobians_raw(pars.size());
    std::vector<double*> jacobians_minimal_raw(pars.size());
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
            Eigen::RowMajor>>> jac(pars.size()), jac_min(pars.size());
    for (size_t j = 0; j < pars.size(); ++j) {
      parameters_raw[j] = pars[j].second->parameters();
      jac[j].resize(rdim, pars[j].second->dimension());
      jacobians_raw[j] = jac[j].data();
      jac_min[j].resize(rdim, pars[j].second->minimalDimension());
      jacobians_minimal_raw[j] = jac_min[j].data();
    }
    res.error_interface_ptr->EvaluateWithMinimalJacobians(
        parameters_raw.data(), residuals_eigen.data(),
        jacobians_raw.data(), jacobians_minimal_raw.data());

    if (res.loss_function_ptr) {
      // Triggs/BANS correction (Eq. 11), identical to MarginalizationError and ceres'
      // corrector.cc, so J^T J below equals ceres::Covariance's robustified Hessian.
      const double sq_norm = residuals_eigen.squaredNorm();
      double rho[3];
      res.loss_function_ptr->Evaluate(sq_norm, rho);
      const double sqrt_rho1 = std::sqrt(rho[1]);
      double alpha_sq_norm = 0.0;
      if (!(sq_norm == 0.0 || rho[2] <= 0.0)) {
        const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
        const double alpha = 1.0 - std::sqrt(D);
        alpha_sq_norm = alpha / sq_norm;
      }
      for (size_t j = 0; j < pars.size(); ++j) {
        jac_min[j] = sqrt_rho1 * (jac_min[j] -
            alpha_sq_norm * residuals_eigen * (residuals_eigen.transpose() * jac_min[j]));
      }
    }

    // Resolve slots and scatter all ordered pairs.
    std::vector<const Slot*> s(pars.size(), nullptr);
    for (size_t j = 0; j < pars.size(); ++j) {
      auto it = slot.find(pars[j].second->id());
      if (it != slot.end()) s[j] = &it->second;
    }
    for (size_t a = 0; a < pars.size(); ++a) {
      if (!s[a]) continue;
      for (size_t b = 0; b < pars.size(); ++b) {
        if (!s[b]) continue;
        scatter(s[a], s[b], (jac_min[a].transpose() * jac_min[b]).eval());
      }
    }
  }

  if (!structure_ok) return fail("partition_violated");
  H_aa = 0.5 * (H_aa + H_aa.transpose()).eval();
  if (n_dim > 0) H_nn = 0.5 * (H_nn + H_nn.transpose()).eval();

  // Propagated linear-algebra error bound of the stage-2 Schur subtraction, in the units
  // of the reduced ambiguity information (set by the iterative-refinement loop below and
  // enforced at the final gate).
  double arith_err = 0.0;

  // ---- Stage 1: eliminate landmark blocks, one 3x3 system at a time. Each landmark's
  // information is rank-truncated at an eps-relative tolerance: a weak-parallax landmark's
  // unobservable direction is dropped exactly (for a PSD Gauss-Newton sum, a null mode of
  // H_ll has identically zero coupling rows, so the Moore-Penrose drop IS the exact
  // marginalization of that mode). This per-block treatment is what keeps ill-conditioned
  // landmarks from poisoning the global elimination -- the root cause of the H-level
  // noise blow-ups observed against the ceres reference before this stage existed. ----
  for (auto& lm : lms) {
    const Eigen::Matrix3d Hll = 0.5 * (lm.Hll + lm.Hll.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> esl(Hll);
    if (esl.info() != Eigen::Success) return fail("landmark_eigen");
    const double lmax = esl.eigenvalues().maxCoeff();
    if (!(lmax > 0.0)) continue;  // landmark with no information: nothing to subtract
    const double ltol = eps * 3.0 * lmax;
    Eigen::Vector3d inv_eig = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
      const double lambda = esl.eigenvalues()(i);
      if (lambda > ltol) inv_eig(i) = 1.0 / lambda;
    }
    const Eigen::Matrix3d V = esl.eigenvectors();
    // H_nn -= B pinv(H_ll) B^T, restricted to the few N-blocks this landmark touches.
    for (size_t i = 0; i < lm.strips.size(); ++i) {
      const int off_i = lm.strips[i].first;
      const Eigen::MatrixXd BiV = lm.strips[i].second * V;  // dim_i x 3
      for (size_t j = 0; j < lm.strips.size(); ++j) {
        const int off_j = lm.strips[j].first;
        const Eigen::MatrixXd BjV = lm.strips[j].second * V;
        H_nn.block(off_i, off_j, BiV.rows(), BjV.rows()).noalias() -=
            BiV * inv_eig.asDiagonal() * BjV.transpose();
      }
    }
  }

  // ---- Stage 2: eliminate the remaining dense nuisance block:
  //   S = H_aa - H_an H_nn'^{-1} H_an^T
  // via dense LDLT (window-bounded size) on the JACOBI-EQUILIBRATED system: H_nn mixes
  // states with wildly different information scales (position vs bias vs clock), so raw
  // condition numbers reflect unit scaling, not degeneracy. Symmetric diagonal scaling
  // Hs = D^-1 H_nn D^-1 with D = sqrt(diag(H_nn)) removes the unit artifact (the same
  // preconditioner pattern MarginalizationError::updateErrorComputation uses); the
  // equilibration cancels exactly in T = rhs_s^T Y = H_an H_nn^-1 H_an^T. Numerical
  // trust is enforced NOT by an a-priori condition-number gate but by iterative
  // refinement (below): the last increment's propagated effect on the Schur term
  // (arith_err) must fall below the reduced information's smallest eigenvalue at the
  // final gate, else we ABSTAIN (caller falls back to the shadow covariance). This is
  // self-validating -- on an eps-unresolvable system the increments do not shrink and
  // the gate rejects. All thresholds are machine-precision-derived, not tuned. ----
  Eigen::MatrixXd S;
  if (n_dim == 0) {
    S = H_aa;  // no nuisance states in the active graph (tiny/degenerate window)
  } else {
    // Jacobi equilibration (guard non-positive diagonals; PSD structure implies such a
    // state's whole row is zero, and the residual/condition gates below handle it).
    Eigen::VectorXd p_inv(n_dim);
    for (int i = 0; i < n_dim; ++i) {
      const double d = H_nn(i, i);
      p_inv(i) = d > 0.0 ? 1.0 / std::sqrt(d) : 1.0;
    }
    const Eigen::MatrixXd Hs =
        p_inv.asDiagonal() * H_nn * p_inv.asDiagonal();
    const Eigen::MatrixXd rhs_s = p_inv.asDiagonal() * H_an.transpose();

    Eigen::LDLT<Eigen::MatrixXd> ldlt(0.5 * (Hs + Hs.transpose()));
    if (ldlt.info() != Eigen::Success) return fail("nuisance_ldlt");
    Eigen::MatrixXd Y = ldlt.solve(rhs_s);
    if (!Y.allFinite()) return fail("nuisance_solve_nonfinite");

    // Iterative refinement -- self-validating arithmetic. Refinement of an LDLT solve
    // converges iff the (equilibrated) system is resolvable in double precision; the last
    // increment dY bounds the remaining linear-algebra error. We propagate that bound to
    // the Schur complement (dT below) and, at the final gate, accept only when it is
    // provably below the reduced information's smallest eigenvalue -- i.e. when the
    // arithmetic error cannot materially change Q_aa. On a genuinely singular or
    // eps-unresolvable system the increments do not shrink and the gate rejects. No
    // tuned constants: the criterion is convergence itself.
    for (int it = 0; it < 3; ++it) {
      const Eigen::MatrixXd R = rhs_s - Hs * Y;
      const Eigen::MatrixXd dY = ldlt.solve(R);
      if (!dY.allFinite()) return fail("refine_nonfinite");
      Y += dY;
      arith_err = (rhs_s.transpose() * dY).norm();
    }
    const Eigen::MatrixXd T = rhs_s.transpose() * Y;  // = H_an H_nn'^-1 H_an^T
    if (!T.allFinite()) return fail("schur_nonfinite");
    S = H_aa - T;
  }
  S = 0.5 * (S + S.transpose()).eval();

  // ---- Invert with a strict positive-definite gate: eps-relative roundoff tolerance
  // plus the propagated linear-algebra error bound (no jitter, no tuned constants). ----
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S);
  if (es.info() != Eigen::Success) return fail("final_eigen");
  const double maxev = es.eigenvalues().maxCoeff();
  if (!(maxev > 0.0) || !S.allFinite()) return fail("final_nonfinite");
  const double tol = std::max(eps * num_amb * maxev, arith_err);
  if (es.eigenvalues().minCoeff() <= tol) return fail("final_below_arith_err");
  Q_aa = es.eigenvectors() * es.eigenvalues().cwiseInverse().asDiagonal()
       * es.eigenvectors().transpose();
  Q_aa = 0.5 * (Q_aa + Q_aa.transpose()).eval();
  if (!Q_aa.allFinite()) return fail("qaa_nonfinite");
  return true;
}

// Check a Jacobian with numeric differences.
bool Graph::isMinimalJacobianCorrect(ceres::ResidualBlockId residual_block_id,
                                   double relTol) const
{
  std::shared_ptr<const ErrorInterface> error_interface_ptr =
      errorInterfacePtr(residual_block_id);
  ParameterBlockCollection parameter_blocks = parameters(residual_block_id);

  // set up data structures for storage
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > >
      J(parameter_blocks.size());
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > >
      J_min(parameter_blocks.size());
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<Eigen::Matrix<double, Eigen::Dynamic,
      Eigen::Dynamic, Eigen::RowMajor> > > J_min_numDiff(
      parameter_blocks.size());
  std::vector<double*> parameters(parameter_blocks.size());
  std::vector<double*> jacobians(parameter_blocks.size());
  std::vector<double*> jacobians_minimal(parameter_blocks.size());
  for (size_t i = 0; i < parameter_blocks.size(); ++i)
  {
    // fill in
    J[i].resize(error_interface_ptr->residualDim(),
                parameter_blocks[i].second->dimension());
    J_min[i].resize(error_interface_ptr->residualDim(),
                    parameter_blocks[i].second->minimalDimension());
    J_min_numDiff[i].resize(error_interface_ptr->residualDim(),
                            parameter_blocks[i].second->minimalDimension());
    parameters[i] = parameter_blocks[i].second->parameters();
    jacobians[i] = J[i].data();
    jacobians_minimal[i] = J_min[i].data();
  }

  // calculate num diff Jacobians
  const double delta = 1e-8;
  for (size_t i = 0; i < parameter_blocks.size(); ++i)
  {
    for (size_t j = 0; j < parameter_blocks[i].second->minimalDimension(); ++j)
    {
      Eigen::VectorXd residuals_p(error_interface_ptr->residualDim());
      Eigen::VectorXd residuals_m(error_interface_ptr->residualDim());

      // apply positive delta
      Eigen::VectorXd parameters_p(parameter_blocks[i].second->dimension());
      Eigen::VectorXd parameters_m(parameter_blocks[i].second->dimension());
      Eigen::VectorXd plus(parameter_blocks[i].second->minimalDimension());
      plus.setZero();
      plus[j] = delta;
      parameter_blocks[i].second->plus(parameters[i], plus.data(),
                                       parameters_p.data());
      parameters[i] = parameters_p.data();
      error_interface_ptr->EvaluateWithMinimalJacobians(parameters.data(),
                                                       residuals_p.data(),
                                                       nullptr,
                                                       nullptr);
      parameters[i] = parameter_blocks[i].second->parameters();  // reset
      // apply negative delta
      plus.setZero();
      plus[j] = -delta;
      parameter_blocks[i].second->plus(parameters[i], plus.data(),
                                       parameters_m.data());
      parameters[i] = parameters_m.data();
      error_interface_ptr->EvaluateWithMinimalJacobians(parameters.data(),
                                                       residuals_m.data(),
                                                       nullptr,
                                                       nullptr);
      parameters[i] = parameter_blocks[i].second->parameters();  // reset
      // calculate numeric difference
      J_min_numDiff[i].col(j) = (residuals_p - residuals_m) * 1.0 / (2.0 * delta);
    }
  }

  // calculate analytic Jacobians and compare
  bool isCorrect = true;
  Eigen::VectorXd residuals(error_interface_ptr->residualDim());
  for (size_t i = 0; i < parameter_blocks.size(); ++i)
  {
    // calc
    error_interface_ptr->EvaluateWithMinimalJacobians(parameters.data(),
                                                     residuals.data(),
                                                     jacobians.data(),
                                                     jacobians_minimal.data());
    // check
    double norm_minimal = J_min_numDiff[i].norm();
    Eigen::MatrixXd J_diff_minimal = J_min_numDiff[i] - J_min[i];
    double max_diff_minimal =
        std::max(-J_diff_minimal.minCoeff(), J_diff_minimal.maxCoeff());

    if (max_diff_minimal / norm_minimal > relTol)
    {
      LOG(INFO) << "Minimal Jacobian inconsistent: "
                << kErrorToStr.at(error_interface_ptr->typeInfo());
      LOG(INFO) << "num diff Jacobian[" << i << "]:\n" << J_min_numDiff[i];
      LOG(INFO) << "provided Jacobian[" << i << "]:\n" << J_min[i];
      LOG(INFO) << "relative error: " << max_diff_minimal / norm_minimal
                << ", relative tolerance: " << relTol;
      isCorrect = false;
    }
  }

  return isCorrect;
}

// Add a parameter block to the map
bool Graph::addParameterBlock(
    std::shared_ptr<ParameterBlock> parameter_block,
    int parameterization, const int /*group*/)
{

  CHECK(parameter_block != nullptr);
  VLOG(200) << "Adding parameter block with parameterization "
            << parameterization << " and id " << BackendId(parameter_block->id());

  // check Id availability
  if (parameterBlockExists(parameter_block->id()))
  {
    LOG(ERROR) << "Parameter block with id " << BackendId(parameter_block->id())
               << " exists already!";
    return false;
  }

  id_to_parameter_block_map_.insert(
      std::pair<uint64_t, std::shared_ptr<ParameterBlock> >(
          parameter_block->id(), parameter_block));

  // also add to ceres problem
  switch (parameterization)
  {
    case Parameterization::Trivial:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension());
      break;
    }
    case Parameterization::HomogeneousPoint:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension(),
                                  &homogeneous_point_local_parameterization_);
      parameter_block->setLocalParameterizationPtr(
          &homogeneous_point_local_parameterization_);
      break;
    }
    case Parameterization::Pose6d:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension(),
                                  &pose_local_parameterization_);
      parameter_block->setLocalParameterizationPtr(&pose_local_parameterization_);
      break;
    }
    default:
    {
      LOG(ERROR) << "Unknown parameterization!";
      return false;
      break;  // just for consistency...
    }
  }

  /*const LocalParamizationAdditionalInterfaces* ptr =
      dynamic_cast<const LocalParamizationAdditionalInterfaces*>(
      parameter_block->localParameterizationPtr());
  if(ptr)
    std::cout<<"verify local size "<< parameter_block->localParameterizationPtr()->LocalSize() << " = "<<
            int(ptr->verify(parameter_block->parameters()))<<
            std::endl;*/

  return true;
}

// Remove a parameter block from the graph.
bool Graph::removeParameterBlock(uint64_t parameter_block_id)
{
  if (!parameterBlockExists(parameter_block_id))
  {
    return false;
  }
  VLOG(200) << "Removing paramter block with ID " << BackendId(parameter_block_id);

  // remove all connected residuals
  const ResidualBlockCollection res = residuals(parameter_block_id);
  for (size_t i = 0; i < res.size(); ++i)
  {
    removeResidualBlock(res[i].residual_block_id);  // remove in ceres and book-keeping
  }
  problem_->RemoveParameterBlock(
      parameterBlockPtr(parameter_block_id)->parameters());  // remove parameter block
  id_to_parameter_block_map_.erase(parameter_block_id);  // remove book-keeping
  return true;
}

// Remove a parameter block from the graph.
bool Graph::removeParameterBlock(
    std::shared_ptr<ParameterBlock> parameter_block)
{
  return removeParameterBlock(parameter_block->id());
}

// Adds a residual block.
ceres::ResidualBlockId Graph::addResidualBlock(
    std::shared_ptr< ceres::CostFunction> cost_function,
    ceres::LossFunction* loss_function,
    std::vector<std::shared_ptr<ParameterBlock> >& parameter_block_ptrs)
{
  ceres::ResidualBlockId return_id;
  std::vector<double*> parameter_blocks;
  ParameterBlockCollection parameter_block_collection;
  for (size_t i = 0; i < parameter_block_ptrs.size(); ++i)
  {
    parameter_blocks.push_back(parameter_block_ptrs.at(i)->parameters());
    parameter_block_collection.push_back(
        ParameterBlockSpec(parameter_block_ptrs.at(i)->id(),
                           parameter_block_ptrs.at(i)));
  }

  // add in ceres
  return_id = problem_->AddResidualBlock(cost_function.get(), loss_function,
                                         parameter_blocks);

  if (FLAGS_v >=200)
  {
    std::stringstream s;
    s << "Adding residual block: "
      << kErrorToStr.at(std::dynamic_pointer_cast<ErrorInterface>(cost_function)->typeInfo())
      << " with id " << return_id
      << " connected to the following parameter blocks:\n";
    for (auto block : parameter_block_ptrs)
    {
      s << BackendId(block->id()) << "\n";
    }
    VLOG(200) << s.str();
  }

  // add in book-keeping
  std::shared_ptr<ErrorInterface> error_interface_ptr =
      std::dynamic_pointer_cast<ErrorInterface>(cost_function);
  CHECK(error_interface_ptr!=0)
      << "Supplied a cost function without ErrorInterface";
  residual_block_id_to_residual_block_spec_map_.insert(
      std::pair< ceres::ResidualBlockId, ResidualBlockSpec>(
          return_id,
          ResidualBlockSpec(return_id, loss_function, error_interface_ptr)));

  // update book-keeping
  bool insertion_success;
  std::tie(std::ignore, insertion_success) =
      residual_block_id_to_parameter_block_collection_map_.insert(
          std::make_pair(return_id, parameter_block_collection));
  if (insertion_success == false)
  {
    return ceres::ResidualBlockId(0);
  }

  // update ResidualBlock pointers on involved ParameterBlocks
  for (uint64_t parameter_id = 0;
      parameter_id < parameter_block_collection.size(); ++parameter_id)
  {
    id_to_residual_block_multimap_.insert(
        std::pair<uint64_t, ResidualBlockSpec>(
            parameter_block_collection[parameter_id].first,
            ResidualBlockSpec(return_id, loss_function, error_interface_ptr)));
  }

  return return_id;
}

// Add a residual block. See respective ceres docu. If more are needed, see other interface.
ceres::ResidualBlockId Graph::addResidualBlock(
    std::shared_ptr< ceres::CostFunction> cost_function,
    ceres::LossFunction* loss_function,
    std::shared_ptr<ParameterBlock> x0,
    std::shared_ptr<ParameterBlock> x1,
    std::shared_ptr<ParameterBlock> x2,
    std::shared_ptr<ParameterBlock> x3,
    std::shared_ptr<ParameterBlock> x4,
    std::shared_ptr<ParameterBlock> x5,
    std::shared_ptr<ParameterBlock> x6,
    std::shared_ptr<ParameterBlock> x7,
    std::shared_ptr<ParameterBlock> x8,
    std::shared_ptr<ParameterBlock> x9)
{

  CHECK(cost_function != nullptr);
  std::vector<std::shared_ptr<ParameterBlock> > parameter_block_ptrs;
  if (x0 != 0)
  {
    parameter_block_ptrs.push_back(x0);
  }
  if (x1 != 0)
  {
    parameter_block_ptrs.push_back(x1);
  }
  if (x2 != 0)
  {
    parameter_block_ptrs.push_back(x2);
  }
  if (x3 != 0)
  {
    parameter_block_ptrs.push_back(x3);
  }
  if (x4 != 0)
  {
    parameter_block_ptrs.push_back(x4);
  }
  if (x5 != 0)
  {
    parameter_block_ptrs.push_back(x5);
  }
  if (x6 != 0)
  {
    parameter_block_ptrs.push_back(x6);
  }
  if (x7 != 0)
  {
    parameter_block_ptrs.push_back(x7);
  }
  if (x8 != 0)
  {
    parameter_block_ptrs.push_back(x8);
  }
  if (x9 != 0)
  {
    parameter_block_ptrs.push_back(x9);
  }

  return Graph::addResidualBlock(cost_function, loss_function, parameter_block_ptrs);

}

// Replace the parameters connected to a residual block ID.
void Graph::resetResidualBlock(
    ceres::ResidualBlockId residual_block_id,
    std::vector<std::shared_ptr<ParameterBlock> >& parameter_block_ptrs)
{
  // remember the residual block spec:
  ResidualBlockSpec spec =
      residual_block_id_to_residual_block_spec_map_[residual_block_id];
  // remove residual from old parameter set
  ResidualBlockIdToParameterBlockCollectionMap::iterator it =
      residual_block_id_to_parameter_block_collection_map_.find(residual_block_id);
  CHECK(it!=residual_block_id_to_parameter_block_collection_map_.end())
      << "residual block not in graph.";
  for (ParameterBlockCollection::iterator parameter_it = it->second.begin();
      parameter_it != it->second.end(); ++parameter_it)
  {
    uint64_t parameter_id = parameter_it->second->id();
    std::pair<IdToResidualBlockMultimap::iterator,
        IdToResidualBlockMultimap::iterator> range = id_to_residual_block_multimap_
        .equal_range(parameter_id);
    CHECK(range.first!=id_to_residual_block_multimap_.end())
        << "book-keeping is broken";
    for (IdToResidualBlockMultimap::iterator it2 = range.first;
        it2 != range.second;)
    {
      if (residual_block_id == it2->second.residual_block_id)
      {
        it2 = id_to_residual_block_multimap_.erase(it2);  // remove book-keeping
      }
      else
      {
        it2++;
      }
    }
  }

  ParameterBlockCollection parameter_block_collection;
  for (size_t i = 0; i < parameter_block_ptrs.size(); ++i)
  {
    parameter_block_collection.push_back(
        ParameterBlockSpec(parameter_block_ptrs.at(i)->id(),
                           parameter_block_ptrs.at(i)));
  }

  // update book-keeping
  it->second = parameter_block_collection;

  // update ResidualBlock pointers on involved ParameterBlocks
  for (uint64_t parameter_id = 0;
      parameter_id < parameter_block_collection.size(); ++parameter_id)
  {
    id_to_residual_block_multimap_.insert(
        std::pair<uint64_t, ResidualBlockSpec>(
            parameter_block_collection[parameter_id].first, spec));
  }
}

// Remove a residual block.
bool Graph::removeResidualBlock(ceres::ResidualBlockId residual_block_id)
{
  VLOG(200) << "Removing residual block with ID " << residual_block_id;

  ResidualBlockIdToParameterBlockCollectionMap::iterator it =
      residual_block_id_to_parameter_block_collection_map_.find(residual_block_id);
  if (it == residual_block_id_to_parameter_block_collection_map_.end())
  {
    LOG(ERROR) << "Residual block " << residual_block_id << " not in graph!";
    return false;
  }

  problem_->RemoveResidualBlock(residual_block_id);  // remove in ceres

  for (ParameterBlockCollection::iterator parameter_it = it->second.begin();
      parameter_it != it->second.end(); ++parameter_it)
  {
    uint64_t parameter_id = parameter_it->second->id();
    std::pair<IdToResidualBlockMultimap::iterator,
        IdToResidualBlockMultimap::iterator> range = id_to_residual_block_multimap_
        .equal_range(parameter_id);
    CHECK(range.first!=id_to_residual_block_multimap_.end())
        << "book-keeping is broken";

    for (IdToResidualBlockMultimap::iterator it2 = range.first;
        it2 != range.second;)
    {
      if (residual_block_id == it2->second.residual_block_id)
      {
        it2 = id_to_residual_block_multimap_.erase(it2);  // remove book-keeping
      }
      else
      {
        it2++;
      }
    }
  }
  residual_block_id_to_parameter_block_collection_map_.erase(it);  // remove book-keeping
  residual_block_id_to_residual_block_spec_map_.erase(residual_block_id);  // remove book-keeping
  return true;
}

// Do not optimise a certain parameter block.
bool Graph::setParameterBlockConstant(uint64_t parameter_block_id)
{
  if (!parameterBlockExists(parameter_block_id))
  {
    return false;
  }
  std::shared_ptr<ParameterBlock> parameter_block = id_to_parameter_block_map_.find(
      parameter_block_id)->second;
  parameter_block->setFixed(true);
  problem_->SetParameterBlockConstant(parameter_block->parameters());
  return true;
}

bool Graph::isParameterBlockConstant(uint64_t parameter_block_id)
{
  if (!parameterBlockExists(parameter_block_id))
  {
    return false;
  }
  std::shared_ptr<ParameterBlock> parameter_block = id_to_parameter_block_map_.find(
      parameter_block_id)->second;
  CHECK_EQ(problem_->IsParameterBlockConstant(parameter_block->parameters()),
           parameter_block->fixed());
  return parameter_block->fixed();
}

// Optimise a certain parameter block (this is the default).
bool Graph::setParameterBlockVariable(uint64_t parameter_block_id)
{
  if (!parameterBlockExists(parameter_block_id))
    return false;
  std::shared_ptr<ParameterBlock> parameter_block =
      id_to_parameter_block_map_.find(parameter_block_id)->second;
  parameter_block->setFixed(false);
  problem_->SetParameterBlockVariable(parameter_block->parameters());
  return true;
}

// Reset the (local) parameterisation of a parameter block.
bool Graph::resetParameterization(uint64_t parameter_block_id,
                                int parameterization)
{
  if (!parameterBlockExists(parameter_block_id))
  {
    return false;
  }
  // the ceres documentation states that a parameterization may never be changed on.
  // therefore, we have to remove the parameter block in question and re-add it.
  ResidualBlockCollection res = residuals(parameter_block_id);
  std::shared_ptr<ParameterBlock> par_block_ptr =
      parameterBlockPtr(parameter_block_id);

  // get parameter block pointers
  std::vector<std::vector<std::shared_ptr<ParameterBlock> > >
      parameter_block_ptrs(res.size());
  for (size_t r = 0; r < res.size(); ++r)
  {
    ParameterBlockCollection pspec = parameters(res[r].residual_block_id);
    for (size_t p = 0; p < pspec.size(); ++p)
    {
      parameter_block_ptrs[r].push_back(pspec[p].second);
    }
  }

  // remove
  // int group = options.linear_solver_ordering->GroupId(parBlockPtr->parameters());
  removeParameterBlock(parameter_block_id);
  // add with new parameterization
  addParameterBlock(par_block_ptr, parameterization/*,group*/);

  // re-assemble
  for (size_t r = 0; r < res.size(); ++r)
  {
    addResidualBlock(
        std::dynamic_pointer_cast< ceres::CostFunction>(
            res[r].error_interface_ptr),
        res[r].loss_function_ptr, parameter_block_ptrs[r]);
  }

  return true;
}

// Set the (local) parameterisation of a parameter block.
bool Graph::setParameterization(
    uint64_t parameter_block_id,
    ceres::LocalParameterization* local_parameterization)
{
  if (!parameterBlockExists(parameter_block_id))
  {
    return false;
  }
  problem_->SetParameterization(
      id_to_parameter_block_map_.find(parameter_block_id)->second->parameters(),
      local_parameterization);
  id_to_parameter_block_map_.find(parameter_block_id)->second
      ->setLocalParameterizationPtr(local_parameterization);
  return true;
}

// getters
// Get a shared pointer to a parameter block.
std::shared_ptr<ParameterBlock> Graph::parameterBlockPtr(
    uint64_t parameter_block_id)
{
  // get a parameterBlock
  CHECK(parameterBlockExists(parameter_block_id))
      << "parameterBlock with id " << BackendId(parameter_block_id)
      << " does not exist";
  if (parameterBlockExists(parameter_block_id))
  {
    return id_to_parameter_block_map_.find(parameter_block_id)->second;
  }
  return std::shared_ptr<ParameterBlock>();  // NULL
}

// Get a shared pointer to a parameter block.
std::shared_ptr<const ParameterBlock> Graph::parameterBlockPtr(
    uint64_t parameter_block_id) const
{
  // get a parameterBlock
  if (parameterBlockExists(parameter_block_id))
  {
    return id_to_parameter_block_map_.find(parameter_block_id)->second;
  }
  return std::shared_ptr<const ParameterBlock>();  // NULL
}

// Get the residual blocks of a parameter block.
Graph::ResidualBlockCollection Graph::residuals(uint64_t parameter_block_id) const
{
  // get the residual blocks of a parameter block
  IdToResidualBlockMultimap::const_iterator it1 = id_to_residual_block_multimap_
      .find(parameter_block_id);
  if (it1 == id_to_residual_block_multimap_.end())
    return Graph::ResidualBlockCollection();  // empty
  ResidualBlockCollection returnResiduals;
  std::pair<IdToResidualBlockMultimap::const_iterator,
      IdToResidualBlockMultimap::const_iterator> range =
      id_to_residual_block_multimap_.equal_range(parameter_block_id);
  for (IdToResidualBlockMultimap::const_iterator it = range.first;
      it != range.second; ++it)
  {
    returnResiduals.push_back(it->second);
  }
  return returnResiduals;
}

// Get a shared pointer to an error term.
std::shared_ptr<ErrorInterface> Graph::errorInterfacePtr(
    ceres::ResidualBlockId residual_block_id)
{  // get a vertex
  ResidualBlockIdToResidualBlockSpecMap::iterator it =
      residual_block_id_to_residual_block_spec_map_.find(residual_block_id);
  if (it == residual_block_id_to_residual_block_spec_map_.end())
  {
    return std::shared_ptr<ErrorInterface>();  // NULL
  }
  return it->second.error_interface_ptr;
}

// Get a shared pointer to an error term.
std::shared_ptr<const ErrorInterface> Graph::errorInterfacePtr(
    ceres::ResidualBlockId residual_block_id) const
{  // get a vertex
  ResidualBlockIdToResidualBlockSpecMap::const_iterator it =
      residual_block_id_to_residual_block_spec_map_.find(residual_block_id);
  if (it == residual_block_id_to_residual_block_spec_map_.end())
  {
    return std::shared_ptr<ErrorInterface>();  // NULL
  }
  return it->second.error_interface_ptr;
}

// Get the parameters of a residual block.
Graph::ParameterBlockCollection Graph::parameters(
    ceres::ResidualBlockId residual_block_id) const
{
  // get the parameter blocks connected
  ResidualBlockIdToParameterBlockCollectionMap::const_iterator it =
      residual_block_id_to_parameter_block_collection_map_.find(residual_block_id);
  if (it == residual_block_id_to_parameter_block_collection_map_.end())
  {
    ParameterBlockCollection empty;
    return empty;  // empty vector
  }
  return it->second;
}

// Get all the residual blocks
Graph::ResidualBlockCollection Graph::residuals() const
{
  ResidualBlockCollection returnResiduals;
  for (auto it : residual_block_id_to_residual_block_spec_map_) {
    returnResiduals.push_back(it.second);
  }
  return returnResiduals;
}

// Get all the parameter blocks
Graph::ParameterBlockCollection Graph::parameters() const
{
  ParameterBlockCollection returnParameters;
  for (auto it : residual_block_id_to_parameter_block_collection_map_) {
    for (auto parameter : it.second) {
      if (std::find(returnParameters.begin(), returnParameters.end(), parameter) 
        != returnParameters.end()) continue;
      returnParameters.push_back(parameter);
    }
  }
  return returnParameters;
}

// Get covariance estimation of given parameter blocks
bool Graph::computeCovariance(
    const std::vector<uint64_t>& parameter_block_ids,
    Eigen::MatrixXd& covariance,
    bool use_dense_svd)
{
  // Get parameters
  std::vector<const double*> parameters;
  std::vector<size_t> parameter_block_sizes;
  std::vector<size_t> parameter_block_starts;
  size_t parameter_size = 0;
  for (size_t i = 0; i < parameter_block_ids.size(); i++) {
    auto it = id_to_parameter_block_map_.find(parameter_block_ids[i]);
    if (it == id_to_parameter_block_map_.end()) {
      LOG(ERROR) << "Parameter block does not exist!";
      return false;
    }
    auto& parameter_block = it->second;
    parameters.push_back(parameter_block->parameters());
    parameter_block_sizes.push_back(parameter_block->dimension());
    parameter_block_starts.push_back(parameter_size);
    parameter_size += parameter_block->dimension();
  }

  // Make pairs
  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  for (size_t i = 0; i < parameters.size(); i++) {
    for (size_t j = i; j < parameters.size(); j++) {
      covariance_blocks.push_back(std::make_pair(parameters[i], parameters[j]));
    }
  }

  const auto extract_covariance =
    [&](const char* method, const ceres::Covariance& covariance_handle,
        Eigen::MatrixXd& output) -> bool {
      output = Eigen::MatrixXd::Zero(parameter_size, parameter_size);
      for (size_t i = 0; i < parameters.size(); i++) {
        for (size_t j = i; j < parameters.size(); j++) {
          size_t size_i = parameter_block_sizes[i];
          size_t size_j = parameter_block_sizes[j];
          size_t start_i = parameter_block_starts[i];
          size_t start_j = parameter_block_starts[j];
          const double* para_i = parameters[i];
          const double* para_j = parameters[j];
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            cov_i_j(size_i, size_j);
          cov_i_j.setZero();
          if (!covariance_handle.GetCovarianceBlock(para_i, para_j,
                                                    cov_i_j.data())) {
            LOG(WARNING) << "Failed to extract covariance block with "
                         << method << ": block " << i << ", " << j;
            return false;
          }
          if (!cov_i_j.allFinite()) {
            LOG(WARNING) << "Non-finite covariance block from "
                         << method << ": block " << i << ", " << j;
            return false;
          }
          output.block(start_i, start_j, size_i, size_j) = cov_i_j;
          output.block(start_j, start_i, size_j, size_i) = cov_i_j.transpose();
        }
      }
      output = 0.5 * (output + output.transpose());
      if (!output.allFinite()) {
        LOG(WARNING) << "Non-finite covariance matrix from " << method << ".";
        return false;
      }
      return true;
    };

  const auto compute_with_options =
    [&](bool dense_svd, Eigen::MatrixXd& output) -> bool {
      ceres::Covariance::Options options;
      const char* method = "SPARSE_QR";
      if (dense_svd) {
        method = "DENSE_SVD";
        options.algorithm_type = ceres::DENSE_SVD;
        // Let Ceres detect and drop all numerically unobservable modes. This
        // gives the Moore-Penrose covariance for gauge/rank-deficient graphs.
        options.null_space_rank = -1;
      }

      ceres::Covariance covariance_handle(options);
      if (!covariance_handle.Compute(covariance_blocks, problem_.get())) {
        return false;
      }
      return extract_covariance(method, covariance_handle, output);
    };

  if (use_dense_svd) {
    if (!compute_with_options(true, covariance)) {
      LOG(WARNING) << "Failed to compute covariance with DENSE_SVD.";
      return false;
    }
    return true;
  }

  if (compute_with_options(false, covariance)) {
    return true;
  }

  LOG(WARNING) << "Sparse covariance failed, retrying with DENSE_SVD "
               << "to handle gauge/rank deficiency.";
  if (compute_with_options(true, covariance)) {
    LOG(INFO) << "Recovered covariance with DENSE_SVD fallback for "
              << parameter_block_ids.size() << " parameter blocks "
              << "(" << parameter_size << " scalar parameters).";
    return true;
  }

  LOG(WARNING) << "Failed to compute covariance with SPARSE_QR and DENSE_SVD.";
  return false;
}

// Evaluate all residual blocks and get total cost
double Graph::computeTotalCost(bool apply_loss_function)
{
  ResidualBlockCollection residual_collection = residuals();
  double total_cost_squared = 0.0;
  for (auto& residual : residual_collection)
  {
    Eigen::VectorXd cost = 
      Eigen::VectorXd::Zero(residual.error_interface_ptr->residualDim());
    problem_->EvaluateResidualBlock(residual.residual_block_id, 
      apply_loss_function, cost.data(), nullptr, nullptr);
    const double cost_norm = cost.norm();
    total_cost_squared += cost_norm * cost_norm;
  }
  return sqrt(total_cost_squared);
}

}  //namespace gici
