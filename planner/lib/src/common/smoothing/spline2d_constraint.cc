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

#include "common/smoothing/spline2d_constraint.h"

#include "common/smoothing/utils/math.h"

namespace common {

Spline2dConstraint::Spline2dConstraint(const std::vector<double>& t_knots,
                                       const uint32_t order)
    : t_knots_(t_knots), spline_order_(order) {
  x_constraint_ = Spline1dConstraint(t_knots_, spline_order_);
  y_constraint_ = Spline1dConstraint(t_knots_, spline_order_);

  spline_param_num_ = spline_order_ + 1;
  total_param_ =
      2 * spline_param_num_ * (static_cast<uint32_t>(t_knots.size()) - 1);

  constraint_matrix_ = Eigen::MatrixXd::Zero(0, 0);
  lower_bound_ = std::vector<double>{};
  upper_bound_ = std::vector<double>{};
}

bool Spline2dConstraint::Add2dBoundary(
    const std::vector<double>& t,
    const vector_Eigen<Eigen::Vector2d>& lower_bound,
    const vector_Eigen<Eigen::Vector2d>& upper_bound) {
  // CHECK_EQ(lower_bound.size(), upper_bound.size());

  size_t boundary_size = lower_bound.size();
  std::vector<double> x_lower_bound(boundary_size),
      x_upper_bound(boundary_size);
  std::vector<double> y_lower_bound(boundary_size),
      y_upper_bound(boundary_size);

  for (uint32_t i = 0; i < boundary_size; ++i) {
    x_lower_bound[i] = lower_bound[i].x();
    x_upper_bound[i] = upper_bound[i].x();
    y_lower_bound[i] = lower_bound[i].y();
    y_upper_bound[i] = upper_bound[i].y();
  }

  bool add_x_constraint = x_constraint_.AddBoundary(t, std::move(x_lower_bound),
                                                    std::move(x_upper_bound));
  bool add_y_constraint = y_constraint_.AddBoundary(t, std::move(y_lower_bound),
                                                    std::move(y_upper_bound));

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dStationLateralBoundary(
    const std::vector<double>& t, const vector_Eigen<Eigen::Vector2d>& ref_xy,
    const std::vector<double>& ref_theta, const std::vector<double>& lon_tol,
    const std::vector<double>& lat_tol) {
  if (t.size() != ref_xy.size() || ref_xy.size() != ref_theta.size() ||
      ref_theta.size() != lon_tol.size() || lon_tol.size() != lat_tol.size()) {
    return false;
  }

  Eigen::MatrixXd sl_constraints =
      Eigen::MatrixXd::Zero(2 * t.size(), total_param_);
  std::vector<double> lower_bound, upper_bound;

  for (uint32_t i = 0; i < t.size(); ++i) {
    const uint32_t index = FindSegStartIndex(t[i]);
    const double d_lateral = SignDistance(ref_xy[i], ref_theta[i]);
    const double d_longitudinal =
        SignDistance(ref_xy[i], ref_theta[i] - M_PI_2);
    const double t_corrected = t[i] - t_knots_[index];

    std::vector<double> lateral_coef = AffineCoef(ref_theta[i], t_corrected);
    std::vector<double> longitudianl_coef =
        AffineCoef(ref_theta[i] - M_PI_2, t_corrected);

    const uint32_t index_offset = index * spline_param_num_;

    for (uint32_t j = 0; j < spline_param_num_; ++j) {
      sl_constraints(2 * i, index_offset + j) = lateral_coef[j];
      sl_constraints(2 * i, index_offset + total_param_ / 2 + j) =
          lateral_coef[spline_param_num_ + j];
      sl_constraints(2 * i + 1, index_offset + j) = longitudianl_coef[j];
      sl_constraints(2 * i + 1, index_offset + total_param_ / 2 + j) =
          longitudianl_coef[spline_param_num_ + j];
    }

    lower_bound.emplace_back(d_lateral - lat_tol[i]);
    lower_bound.emplace_back(d_longitudinal - lon_tol[i]);

    upper_bound.emplace_back(d_lateral + lat_tol[i]);
    upper_bound.emplace_back(d_longitudinal + lon_tol[i]);
  }

  return coxy_constraint_.AddConstraint(sl_constraints, lower_bound,
                                        upper_bound);
}

bool Spline2dConstraint::Add2dDerivativeBoundary(
    const std::vector<double>& t,
    const vector_Eigen<Eigen::Vector2d>& lower_bound,
    const vector_Eigen<Eigen::Vector2d>& upper_bound) {
  // CHECK_EQ(lower_bound.size(), upper_bound.size());

  size_t boundary_size = lower_bound.size();
  std::vector<double> x_lower_bound(boundary_size),
      x_upper_bound(boundary_size);
  std::vector<double> y_lower_bound(boundary_size),
      y_upper_bound(boundary_size);

  for (uint32_t i = 0; i < boundary_size; ++i) {
    x_lower_bound[i] = lower_bound[i].x();
    x_upper_bound[i] = upper_bound[i].x();
    y_lower_bound[i] = lower_bound[i].y();
    y_upper_bound[i] = upper_bound[i].y();
  }

  bool add_x_constraint = x_constraint_.AddDerivativeBoundary(
      t, std::move(x_lower_bound), std::move(x_upper_bound));
  bool add_y_constraint = y_constraint_.AddDerivativeBoundary(
      t, std::move(y_lower_bound), std::move(y_upper_bound));

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dSecondDerivativeBoundary(
    const std::vector<double>& t,
    const vector_Eigen<Eigen::Vector2d>& lower_bound,
    const vector_Eigen<Eigen::Vector2d>& upper_bound) {
  // CHECK_EQ(lower_bound.size(), upper_bound.size());

  size_t boundary_size = lower_bound.size();
  std::vector<double> x_lower_bound(boundary_size),
      x_upper_bound(boundary_size);
  std::vector<double> y_lower_bound(boundary_size),
      y_upper_bound(boundary_size);

  for (uint32_t i = 0; i < boundary_size; ++i) {
    x_lower_bound[i] = lower_bound[i].x();
    x_upper_bound[i] = upper_bound[i].x();
    y_lower_bound[i] = lower_bound[i].y();
    y_upper_bound[i] = upper_bound[i].y();
  }

  bool add_x_constraint = x_constraint_.AddSecondDerivativeBoundary(
      t, std::move(x_lower_bound), std::move(x_upper_bound));
  bool add_y_constraint = y_constraint_.AddSecondDerivativeBoundary(
      t, std::move(y_lower_bound), std::move(y_upper_bound));

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dThirdDerivativeBoundary(
    const std::vector<double>& t,
    const vector_Eigen<Eigen::Vector2d>& lower_bound,
    const vector_Eigen<Eigen::Vector2d>& upper_bound) {
  // CHECK_EQ(lower_bound.size(), upper_bound.size());

  size_t boundary_size = lower_bound.size();
  std::vector<double> x_lower_bound(boundary_size),
      x_upper_bound(boundary_size);
  std::vector<double> y_lower_bound(boundary_size),
      y_upper_bound(boundary_size);

  for (uint32_t i = 0; i < boundary_size; ++i) {
    x_lower_bound[i] = lower_bound[i].x();
    x_upper_bound[i] = upper_bound[i].x();
    y_lower_bound[i] = lower_bound[i].y();
    y_upper_bound[i] = upper_bound[i].y();
  }

  bool add_x_constraint = x_constraint_.AddThirdDerivativeBoundary(
      t, std::move(x_lower_bound), std::move(x_upper_bound));
  bool add_y_constraint = y_constraint_.AddThirdDerivativeBoundary(
      t, std::move(y_lower_bound), std::move(y_upper_bound));

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dPointConstraint(const double t,
                                              const Eigen::Vector2d ft) {
  bool add_x_constraint = x_constraint_.AddPointConstraint(t, ft.x());
  bool add_y_constraint = y_constraint_.AddPointConstraint(t, ft.y());

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dPointDerivativeConstraint(
    const double t, const Eigen::Vector2d dft) {
  bool add_x_constraint =
      x_constraint_.AddPointDerivativeConstraint(t, dft.x());
  bool add_y_constraint =
      y_constraint_.AddPointDerivativeConstraint(t, dft.y());

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dPointSecondDerivativeConstraint(
    const double t, const Eigen::Vector2d ddft) {
  bool add_x_constraint =
      x_constraint_.AddPointSecondDerivativeConstraint(t, ddft.x());
  bool add_y_constraint =
      y_constraint_.AddPointSecondDerivativeConstraint(t, ddft.y());

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dPointThirdDerivativeConstraint(
    const double t, const Eigen::Vector2d dddft) {
  bool add_x_constraint =
      x_constraint_.AddPointThirdDerivativeConstraint(t, dddft.x());
  bool add_y_constraint =
      y_constraint_.AddPointThirdDerivativeConstraint(t, dddft.y());

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dSmoothConstraint() {
  bool add_x_constraint = x_constraint_.AddSmoothConstraint();
  bool add_y_constraint = y_constraint_.AddSmoothConstraint();

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dDerivativeSmoothConstraint() {
  bool add_x_constraint = x_constraint_.AddDerivativeSmoothConstraint();
  bool add_y_constraint = y_constraint_.AddDerivativeSmoothConstraint();

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dSecondDerivativeSmoothConstraint() {
  bool add_x_constraint = x_constraint_.AddSecondDerivativeSmoothConstraint();
  bool add_y_constraint = y_constraint_.AddSecondDerivativeSmoothConstraint();

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::Add2dThirdDerivativeSmoothConstraint() {
  bool add_x_constraint = x_constraint_.AddThirdDerivativeSmoothConstraint();
  bool add_y_constraint = y_constraint_.AddThirdDerivativeSmoothConstraint();

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::AddPointAngleConstraint(const double t,
                                                 const double angle) {
  const uint32_t index = FindSegStartIndex(t);
  const double t_corrected = t - t_knots_[index];
  Eigen::MatrixXd angle_constraint = Eigen::MatrixXd::Zero(1, total_param_);
  Eigen::MatrixXd derivative_contraint =
      Eigen::MatrixXd::Zero(1, total_param_ / 2);

  double x_coef = std::sin(angle);
  double y_coef = -std::cos(angle);
  double coef = 1.0;

  for (uint32_t i = 1; i < spline_param_num_; ++i) {
    angle_constraint(0, index * spline_param_num_ + i) = x_coef * i;
    angle_constraint(0, index * spline_param_num_ + total_param_ / 2 + i) =
        y_coef * i;
    derivative_contraint(0, index * spline_param_num_ + i) = coef * i;

    x_coef *= t_corrected;
    y_coef *= t_corrected;
    coef *= t_corrected;
  }

  coxy_constraint_.AddConstraint(angle_constraint, std::vector<double>{0.0},
                                 std::vector<double>{0.0});

  int dx_sign = 1;
  int dy_sign = 1;
  double normalized_angle = fmod(angle, M_PI * 2);
  if (normalized_angle < 0) {
    normalized_angle += M_PI * 2;
  }

  if (normalized_angle > (M_PI / 2) && normalized_angle < (M_PI * 1.5)) {
    dx_sign = -1;
  }

  if (normalized_angle >= M_PI) {
    dy_sign = -1;
  }

  bool add_x_constraint = x_constraint_.AddConstraint(
      derivative_contraint * dx_sign, std::vector<double>{0.0},
      std::vector<double>{udrive_inf});
  bool add_y_constraint = y_constraint_.AddConstraint(
      derivative_contraint * dy_sign, std::vector<double>{0.0},
      std::vector<double>{udrive_inf});

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::AddPointVelocityConstraint(const double t,
                                                    const double fabs_v,
                                                    const int gear,
                                                    const double heading) {
  double normalized_angle = 0.0;
  if (gear > 0) {
    normalized_angle = NormalizeAngle(heading);
  } else {
    normalized_angle = NormalizeAngle(heading + M_PI);
  }
  /* minimum planning velocity should be above control deadzone */
  double normalized_v = std::max(fabs_v, 0.1);
  double vx = normalized_v * std::cos(normalized_angle);
  double vy = normalized_v * std::sin(normalized_angle);

  bool add_x_constraint = x_constraint_.AddPointDerivativeConstraint(t, vx);
  bool add_y_constraint = y_constraint_.AddPointDerivativeConstraint(t, vy);

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::AddPointAccelerationConstraint(
    const double t, const double a, const int gear, const double a_direction) {
  double ax = a * std::cos(a_direction);
  double ay = a * std::sin(a_direction);

  bool add_x_constraint =
      x_constraint_.AddPointSecondDerivativeConstraint(t, ax);
  bool add_y_constraint =
      y_constraint_.AddPointSecondDerivativeConstraint(t, ay);

  return add_x_constraint && add_y_constraint;
}

bool Spline2dConstraint::AddPointKineticBicycleModelConstraint(
    const double t, const Eigen::Vector2d position, const double v,
    const double a, const double steer, const double heading,
    const double wheel_base) {
  double vx = v * std::cos(heading);
  double vy = v * std::sin(heading);
  double ax = a * std::cos(heading) -
              v * v * std::sin(heading) * std::tan(steer) / wheel_base;
  double ay = a * std::sin(heading) +
              v * v * std::cos(heading) * std::tan(steer) / wheel_base;

  bool add_x_constraint =
      x_constraint_.AddPointConstraint(t, position.x()) &&
      x_constraint_.AddPointDerivativeConstraint(t, vx) &&
      x_constraint_.AddPointSecondDerivativeConstraint(t, ax);
  bool add_y_constraint =
      y_constraint_.AddPointConstraint(t, position.y()) &&
      y_constraint_.AddPointDerivativeConstraint(t, vy) &&
      y_constraint_.AddPointSecondDerivativeConstraint(t, ay);

  return add_x_constraint && add_y_constraint;
}

size_t Spline2dConstraint::FindSegStartIndex(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin(), t_knots_.end(), t);
  return std::min<size_t>(upper_bound - t_knots_.begin() - 1,
                          t_knots_.size() - 2);
}

double Spline2dConstraint::SignDistance(const Eigen::Vector2d& ref_point,
                                        const double ref_angle) const {
  return ref_point.x() * (-std::sin(ref_angle)) +
         ref_point.y() * std::cos(ref_angle);
}

std::vector<double> Spline2dConstraint::AffineCoef(const double angle,
                                                   const double t) const {
  const uint32_t num_params = spline_order_ + 1;
  std::vector<double> result(num_params * 2, 0.0);
  double x_coef = -std::sin(angle);
  double y_coef = std::cos(angle);
  for (uint32_t i = 0; i < num_params; ++i) {
    result[i] = x_coef;
    result[i + num_params] = y_coef;
    x_coef *= t;
    y_coef *= t;
  }
  return result;
}

const Eigen::MatrixXd& Spline2dConstraint::constraint_matrix() {
  uint32_t ind_constraints = x_constraint_.constraint_matrix().rows() +
                             y_constraint_.constraint_matrix().rows();
  uint32_t coo_constraints = coxy_constraint_.constraint_matrix().rows();

  constraint_matrix_ =
      Eigen::MatrixXd::Zero(ind_constraints + coo_constraints, total_param_);

  if (ind_constraints) {
    constraint_matrix_.block(0, 0, ind_constraints / 2, total_param_ / 2) =
        x_constraint_.constraint_matrix();
    constraint_matrix_.block(ind_constraints / 2, total_param_ / 2,
                             ind_constraints / 2, total_param_ / 2) =
        y_constraint_.constraint_matrix();
  }
  if (coo_constraints) {
    constraint_matrix_.block(ind_constraints, 0, coo_constraints,
                             total_param_) =
        coxy_constraint_.constraint_matrix();
  }

  return constraint_matrix_;
}

const std::vector<double>& Spline2dConstraint::lower_bound() {
  const auto& x_lower_bound = x_constraint_.affine_constraint().lower_bound();
  const auto& y_lower_bound = y_constraint_.affine_constraint().lower_bound();

  const auto& coo_lower_bound = coxy_constraint_.lower_bound();

  lower_bound_ = x_lower_bound;
  lower_bound_.insert(lower_bound_.end(), y_lower_bound.begin(),
                      y_lower_bound.end());
  lower_bound_.insert(lower_bound_.end(), coo_lower_bound.begin(),
                      coo_lower_bound.end());

  return lower_bound_;
}

const std::vector<double>& Spline2dConstraint::upper_bound() {
  const auto& x_upper_bound = x_constraint_.affine_constraint().upper_bound();
  const auto& y_upper_bound = y_constraint_.affine_constraint().upper_bound();

  const auto& coo_upper_bound = coxy_constraint_.upper_bound();

  upper_bound_ = x_upper_bound;
  upper_bound_.insert(upper_bound_.end(), y_upper_bound.begin(),
                      y_upper_bound.end());
  upper_bound_.insert(upper_bound_.end(), coo_upper_bound.begin(),
                      coo_upper_bound.end());

  return upper_bound_;
}

const AffineConstraint& Spline2dConstraint::x_affine_constraint() {
  return x_constraint_.affine_constraint();
}

const AffineConstraint& Spline2dConstraint::y_affine_constraint() {
  return y_constraint_.affine_constraint();
}
}  // namespace common
