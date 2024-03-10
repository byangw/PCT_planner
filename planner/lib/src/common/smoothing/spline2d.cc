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

#include "common/smoothing/spline2d.h"

#include "common/smoothing/utils/math.h"

namespace common {

Spline2d::Spline2d(const std::vector<double>& t_knots, const uint32_t order)
    : t_knots_(t_knots), spline_order_(order) {
  if (t_knots.size() > 1) {
    for (uint32_t i = 1; i < t_knots_.size(); ++i) {
      splines_.emplace_back(spline_order_);
    }
  }
}

Eigen::Vector2d Spline2d::pos(const double t) const {
  uint32_t index = find_index(t);
  const double param = t - t_knots_[index];
  return Eigen::Vector2d(splines_[index].x(param), splines_[index].y(param));
}

double Spline2d::theta(const double t) const {
  uint32_t index = find_index(t);
  const double param = t - t_knots_[index];
  return std::atan2(splines_[index].DerivativeY(param),
                    splines_[index].DerivativeX(param));
}

void Spline2d::GetCurvature(const double t, double* kappa,
                            double* dkappa) const {
  if (splines_.empty()) return;
  uint32_t index = find_index(t);
  const double param = t - t_knots_[index];
  const double dx = splines_[index].DerivativeX(param);
  const double ddx = splines_[index].SecondDerivativeX(param);
  const double dddx = splines_[index].ThirdDerivativeX(param);
  const double dy = splines_[index].DerivativeY(param);
  const double ddy = splines_[index].SecondDerivativeX(param);
  const double dddy = splines_[index].ThirdDerivativeX(param);
  *kappa = Curvature(dx, ddx, dy, ddy);
  *dkappa = CurvatureDerivative(dx, ddx, dddx, dy, ddy, dddy);
}

double Spline2d::GetCurvature(const double t) const {
  if (splines_.empty()) return 0.0;
  uint32_t index = find_index(t);
  const double param = t - t_knots_[index];
  const double dx = splines_[index].DerivativeX(param);
  const double ddx = splines_[index].SecondDerivativeX(param);
  const double dy = splines_[index].DerivativeY(param);
  const double ddy = splines_[index].SecondDerivativeX(param);
  return Curvature(dx, ddx, dy, ddy);
}

std::pair<double, double> Spline2d::operator()(const double t) const {
  if (splines_.empty()) {
    return std::make_pair(0.0, 0.0);
  }
  uint32_t index = find_index(t);
  return splines_[index](t - t_knots_[index]);
}

double Spline2d::x(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].x(t - t_knots_[index]);
}

double Spline2d::y(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].y(t - t_knots_[index]);
}

double Spline2d::DerivativeX(const double t) const {
  // zero order spline
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].DerivativeX(t - t_knots_[index]);
}

double Spline2d::DerivativeY(const double t) const {
  // zero order spline
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].DerivativeY(t - t_knots_[index]);
}

double Spline2d::Derivative(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].DerivativeY(t - t_knots_[index]) /
         splines_[index].DerivativeX(t - t_knots_[index]);
}

double Spline2d::SecondDerivativeX(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].SecondDerivativeX(t - t_knots_[index]);
}

double Spline2d::SecondDerivativeY(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].SecondDerivativeY(t - t_knots_[index]);
}

double Spline2d::ThirdDerivativeX(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].ThirdDerivativeX(t - t_knots_[index]);
}

double Spline2d::ThirdDerivativeY(const double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  uint32_t index = find_index(t);
  return splines_[index].ThirdDerivativeY(t - t_knots_[index]);
}
/**
 *   @brief: set splines
 **/
bool Spline2d::set_splines(const Eigen::MatrixXd& params,
                           const uint32_t order) {
  const uint32_t num_params = order + 1;
  const uint32_t num_seg = t_knots_.size() - 1;
  // check if the parameter size fit
  if (2 * num_seg * num_params != static_cast<uint32_t>(params.rows())) {
    return false;
  }
  const uint32_t offset = num_seg * num_params;
  for (uint32_t i = 0; i < splines_.size(); ++i) {
    std::vector<double> spline_piece_x(num_params, 0.0);
    std::vector<double> spline_piece_y(num_params, 0.0);
    for (uint32_t j = 0; j < num_params; ++j) {
      spline_piece_x[j] = params(i * num_params + j, 0);
      spline_piece_y[j] = params(i * num_params + offset + j, 0);
    }
    splines_[i].SetParams(spline_piece_x, spline_piece_y);
  }
  spline_order_ = order;
  return true;
}

const Spline2dSeg& Spline2d::smoothing_spline(const uint32_t index) const {
  return splines_[index];
}

const std::vector<double>& Spline2d::t_knots() const { return t_knots_; }

uint32_t Spline2d::spline_order() const { return spline_order_; }

uint32_t Spline2d::find_index(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin(), t_knots_.end(), t);
  return std::min<size_t>(upper_bound - t_knots_.begin() - 1,
                          t_knots_.size() - 2);
}

}  // namespace common
