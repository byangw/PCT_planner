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
#include <algorithm>
#include <vector>

#include "common/smoothing/spline1d_seg.h"

namespace common {

class Spline1d {
 public:
  Spline1d() = default;
  Spline1d(const std::vector<double>& x_knots, const uint32_t order);

  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  bool set_splines(const Eigen::MatrixXd& param_matrix, const uint32_t order);

  const std::vector<double>& x_knots() const;
  uint32_t spline_order() const;

  const std::vector<Spline1dSeg>& splines() const;

 private:
  size_t FindSegStartIndex(const double x) const;

 private:
  std::vector<Spline1dSeg> splines_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
};
}  // namespace common
