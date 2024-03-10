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

#include "common/base/type.h"

#define udrive_inf ((double)1e30)  // NOLINT

namespace common {
/**
 * @class AffineConstraint
 * @brief constraint in form of "l <= Ax <= u", equality constraint means l = u
 */
class AffineConstraint {
 public:
  AffineConstraint() = default;

  AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);

  const Eigen::MatrixXd& constraint_matrix() const;

  const std::vector<double>& lower_bound() const;

  const std::vector<double>& upper_bound() const;

  bool AddConstraint(const Eigen::MatrixXd& constraint_matrix,
                     const std::vector<double>& lower_bound,
                     const std::vector<double>& upper_bound);

  void Print() const;

 private:
  Eigen::MatrixXd constraint_matrix_;

  std::vector<double> lower_bound_;

  std::vector<double> upper_bound_;
};
}  // namespace common
