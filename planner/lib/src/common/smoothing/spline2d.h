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
#include <utility>
#include <vector>

#include "common/smoothing/spline2d_seg.h"

namespace common {

class Spline2d {
 public:
  Spline2d() = default;
  Spline2d(const std::vector<double>& t_knots, const uint32_t order);
  Eigen::Vector2d pos(const double t) const;
  double theta(const double t) const;
  std::pair<double, double> operator()(const double t) const;
  double x(const double t) const;
  double y(const double t) const;
  double DerivativeX(const double t) const;
  double DerivativeY(const double t) const;
  double Derivative(const double t) const;
  double SecondDerivativeX(const double t) const;
  double SecondDerivativeY(const double t) const;
  double ThirdDerivativeX(const double t) const;
  double ThirdDerivativeY(const double t) const;
  bool set_splines(const Eigen::MatrixXd& params, const uint32_t order);
  const Spline2dSeg& smoothing_spline(const uint32_t index) const;
  const std::vector<double>& t_knots() const;
  uint32_t spline_order() const;

  void GetCurvature(const double t, double* kappa, double* dkappa) const;
  double GetCurvature(const double t) const;

 private:
  uint32_t find_index(const double x) const;

 private:
  std::vector<Spline2dSeg> splines_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
};

class Spline2dPoint {
 public:
  Spline2dPoint() = default;
  Spline2dPoint(const Spline2d& spline2d, const double t)
      : fx_(spline2d.x(t)),
        fy_(spline2d.y(t)),
        dfx_(spline2d.DerivativeX(t)),
        dfy_(spline2d.DerivativeY(t)),
        ddfx_(spline2d.SecondDerivativeX(t)),
        ddfy_(spline2d.SecondDerivativeY(t)),
        dddfx_(spline2d.ThirdDerivativeX(t)),
        dddfy_(spline2d.ThirdDerivativeY(t)) {}

  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double dfx() const { return dfx_; }
  double dfy() const { return dfy_; }
  double ddfx() const { return ddfx_; }
  double ddfy() const { return ddfy_; }
  double dddfx() const { return dddfx_; }
  double dddfy() const { return dddfy_; }

 private:
  double fx_ = 0.0;
  double fy_ = 0.0;
  double dfx_ = 0.0;
  double dfy_ = 0.0;
  double ddfx_ = 0.0;
  double ddfy_ = 0.0;
  double dddfx_ = 0.0;
  double dddfy_ = 0.0;
};
}  // namespace common
