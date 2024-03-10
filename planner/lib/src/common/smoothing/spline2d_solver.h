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

#include <Eigen/Core>
#include <vector>

#include "common/smoothing/spline2d.h"
#include "common/smoothing/spline2d_constraint.h"
#include "common/smoothing/spline2d_kernel.h"

namespace common {

class Spline2dSolver {
 public:
  Spline2dSolver() = default;
  Spline2dSolver(const std::vector<double>& t_knots, const uint32_t order)
      : spline_(t_knots, order),
        kernel_(t_knots, order),
        constraint_(t_knots, order) {}

  virtual ~Spline2dSolver() {}

  virtual void Reset(const std::vector<double>& t_knots,
                     const uint32_t order) = 0;

  // customize setup
  virtual Spline2dConstraint* mutable_constraint() = 0;
  virtual Spline2dKernel* mutable_kernel() = 0;
  virtual Spline2d* mutable_spline() = 0;

  // solve
  virtual bool Solve() = 0;

  // extract
  virtual const Spline2d& spline() const = 0;

 protected:
  Spline2d spline_;
  Spline2dKernel kernel_;
  Spline2dConstraint constraint_;
};
}  // namespace common
