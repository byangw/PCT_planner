/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/smoothing/osqp_spline2d_solver.h"

//#include <glog/logging.h>

#include <algorithm>

#include "common/smoothing/solver/osqp/osqp_interface.h"

namespace common {

OsqpSpline2dSolver::OsqpSpline2dSolver(const std::vector<double>& t_knots,
                                       const uint32_t order)
    : Spline2dSolver(t_knots, order) {}

void OsqpSpline2dSolver::Reset(const std::vector<double>& t_knots,
                               const uint32_t order) {
  spline_ = Spline2d(t_knots, order);
  kernel_ = Spline2dKernel(t_knots, order);
  constraint_ = Spline2dConstraint(t_knots, order);
}

Spline2dConstraint* OsqpSpline2dSolver::mutable_constraint() {
  return &constraint_;
}

Spline2dKernel* OsqpSpline2dSolver::mutable_kernel() { return &kernel_; }

Spline2d* OsqpSpline2dSolver::mutable_spline() { return &spline_; }

bool OsqpSpline2dSolver::Solve() {
  const Eigen::MatrixXd& P = kernel_.kernel_matrix();
  const Eigen::MatrixXd& A = constraint_.constraint_matrix();

  Eigen::VectorXd q = kernel_.gradient();

  const size_t n = A.rows();
  // have to copy once anyway
  std::vector<double> lower_bound(constraint_.lower_bound());
  std::vector<double> upper_bound(constraint_.upper_bound());

  // Eigen::Map does not create its own memory
  Eigen::VectorXd l =
      Eigen::Map<Eigen::VectorXd, Eigen::Aligned>(lower_bound.data(), n);
  Eigen::VectorXd u =
      Eigen::Map<Eigen::VectorXd, Eigen::Aligned>(upper_bound.data(), n);

  Eigen::VectorXd solution;

  if (!OsqpInterface::Solve(P, q, A, l, u, &solution)) {
    // LOG(ERROR) << "fail to solve the spline";
    return false;
  }

  return spline_.set_splines(solution, spline_.spline_order());
}

// extract
const Spline2d& OsqpSpline2dSolver::spline() const { return spline_; }

}  // namespace common
