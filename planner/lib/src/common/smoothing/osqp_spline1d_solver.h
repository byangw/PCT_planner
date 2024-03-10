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

#pragma once

#include <vector>

#include "common/smoothing/spline1d_solver.h"

namespace common {

class OsqpSpline1dSolver final : public Spline1dSolver {
 public:
  OsqpSpline1dSolver(const std::vector<double>& t_knots, const uint32_t order);

  ~OsqpSpline1dSolver() = default;

  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline1dConstraint* mutable_constraint() override;
  Spline1dKernel* mutable_kernel() override;
  Spline1d* mutable_spline() override;

  // solve
  bool Solve() override;

  // extract
  const Spline1d& spline() const override;
};

}  // namespace common
