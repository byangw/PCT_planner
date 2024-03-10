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

#include "common/smoothing/spline1d_constraint.h"

namespace common {

Spline1dConstraint::Spline1dConstraint(const std::vector<double>& x_knots,
                                       const uint32_t order)
    : x_knots_(x_knots), spline_order_(order) {
  spline_param_num_ = spline_order_ + 1;
  columns_ = (x_knots_.size() - 1) * spline_param_num_;
}

bool Spline1dConstraint::AddConstraint(const Eigen::MatrixXd constraint,
                                       const std::vector<double>& lower_bound,
                                       const std::vector<double>& upper_bound) {
  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddBoundary(const std::vector<double>& x,
                                     const std::vector<double>& lower_bound,
                                     const std::vector<double>& upper_bound) {
  size_t constraint_size = x.size();
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(constraint_size, columns_);

  for (size_t i = 0; i < constraint_size; ++i) {
    const size_t index = FindSegStartIndex(x[i]);
    const double corrected_x = x[i] - x_knots_[index];
    double coef = 1.0;
    for (size_t j = 0; j < spline_param_num_; ++j) {
      constraint(i, j + index * spline_param_num_) = coef;
      coef *= corrected_x;
    }
  }

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddDerivativeBoundary(
    const std::vector<double>& x, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  size_t constraint_size = x.size();
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(constraint_size, columns_);

  for (size_t i = 0; i < constraint_size; ++i) {
    const size_t index = FindSegStartIndex(x[i]);
    const double corrected_x = x[i] - x_knots_[index];
    double coef = 1.0;
    for (size_t j = 1; j < spline_param_num_; ++j) {
      constraint(i, j + index * spline_param_num_) = coef * j;
      coef *= corrected_x;
    }
  }

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddSecondDerivativeBoundary(
    const std::vector<double>& x, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  size_t constraint_size = x.size();
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(constraint_size, columns_);

  for (size_t i = 0; i < constraint_size; ++i) {
    const size_t index = FindSegStartIndex(x[i]);
    const double corrected_x = x[i] - x_knots_[index];
    double coef = 1.0;
    for (size_t j = 2; j < spline_param_num_; ++j) {
      constraint(i, j + index * spline_param_num_) = coef * j * (j - 1);
      coef *= corrected_x;
    }
  }

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddThirdDerivativeBoundary(
    const std::vector<double>& x, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound) {
  size_t constraint_size = x.size();
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(constraint_size, columns_);

  for (size_t i = 0; i < constraint_size; ++i) {
    const size_t index = FindSegStartIndex(x[i]);
    const double corrected_x = x[i] - x_knots_[index];
    double coef = 1.0;
    for (size_t j = 3; j < spline_param_num_; ++j) {
      constraint(i, j + index * spline_param_num_) =
          coef * j * (j - 1) * (j - 2);
      coef *= corrected_x;
    }
  }

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddPointConstraint(const double x, const double fx) {
  return AddBoundary(std::vector<double>{x}, std::vector<double>{fx},
                     std::vector<double>{fx});
}

bool Spline1dConstraint::AddPointDerivativeConstraint(const double x,
                                                      const double dfx) {
  return AddDerivativeBoundary(std::vector<double>{x}, std::vector<double>{dfx},
                               std::vector<double>{dfx});
}

bool Spline1dConstraint::AddPointSecondDerivativeConstraint(const double x,
                                                            const double ddfx) {
  return AddSecondDerivativeBoundary(std::vector<double>{x},
                                     std::vector<double>{ddfx},
                                     std::vector<double>{ddfx});
}

bool Spline1dConstraint::AddPointThirdDerivativeConstraint(const double x,
                                                           const double dddfx) {
  return AddThirdDerivativeBoundary(std::vector<double>{x},
                                    std::vector<double>{dddfx},
                                    std::vector<double>{dddfx});
}

bool Spline1dConstraint::AddSmoothConstraint() {
  if (x_knots_.size() <= 2) return false;

  size_t n_constraint = x_knots_.size() - 2;
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(n_constraint, columns_);

  for (size_t i = 0; i < n_constraint; ++i) {
    double left_coef = 1.0;
    double right_coef = -1.0;

    const double x_left = x_knots_[i + 1] - x_knots_[i];
    const double x_right = 0.0;
    for (size_t j = 0; j < spline_param_num_; ++j) {
      constraint(i, j + i * spline_param_num_) = left_coef;
      constraint(i, j + (i + 1) * spline_param_num_) = right_coef;
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }

  std::vector<double> lower_bound = std::vector<double>(n_constraint, 0.0),
                      upper_bound = std::vector<double>(n_constraint, 0.0);

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddDerivativeSmoothConstraint() {
  if (x_knots_.size() <= 2) return false;

  size_t n_constraint = (x_knots_.size() - 2) * 2;
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(n_constraint, columns_);

  for (size_t i = 0; i < n_constraint; i += 2) {
    double left_coef = 1.0;
    double left_dcoef = 1.0;
    double right_coef = -1.0;
    double right_dcoef = -1.0;

    const double x_left = x_knots_[i / 2 + 1] - x_knots_[i / 2];
    const double x_right = 0.0;
    for (size_t j = 0; j < spline_param_num_; ++j) {
      constraint(i, spline_param_num_ * i / 2 + j) = left_coef;
      constraint(i, spline_param_num_ * (i / 2 + 1) + j) = right_coef;
      if (j >= 1) {
        constraint(i + 1, spline_param_num_ * i / 2 + j) = left_dcoef * j;
        constraint(i + 1, spline_param_num_ * (i / 2 + 1) + j) =
            right_dcoef * j;
        left_dcoef = left_coef;
        right_dcoef = right_coef;
      }
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }

  std::vector<double> lower_bound = std::vector<double>(n_constraint, 0.0),
                      upper_bound = std::vector<double>(n_constraint, 0.0);

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddSecondDerivativeSmoothConstraint() {
  if (x_knots_.size() <= 2) return false;

  size_t n_constraint = (x_knots_.size() - 2) * 3;
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(n_constraint, columns_);

  for (size_t i = 0; i < n_constraint; i += 3) {
    double left_coef = 1.0;
    double left_dcoef = 1.0;
    double left_ddcoef = 1.0;
    double right_coef = -1.0;
    double right_dcoef = -1.0;
    double right_ddcoef = -1.0;

    const double x_left = x_knots_[i / 3 + 1] - x_knots_[i / 3];
    const double x_right = 0.0;
    for (size_t j = 0; j < spline_param_num_; ++j) {
      constraint(i, spline_param_num_ * i / 3 + j) = left_coef;
      constraint(i, spline_param_num_ * (i / 3 + 1) + j) = right_coef;

      if (j >= 2) {
        constraint(i + 2, spline_param_num_ * i / 3 + j) =
            left_ddcoef * j * (j - 1);
        constraint(i + 2, spline_param_num_ * (i / 3 + 1) + j) =
            right_ddcoef * j * (j - 1);
        left_ddcoef = left_dcoef;
        right_ddcoef = right_dcoef;
      }

      if (j >= 1) {
        constraint(i + 1, spline_param_num_ * i / 3 + j) = left_dcoef * j;
        constraint(i + 1, spline_param_num_ * (i / 3 + 1) + j) =
            right_dcoef * j;
        left_dcoef = left_coef;
        right_dcoef = right_coef;
      }
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }

  std::vector<double> lower_bound = std::vector<double>(n_constraint, 0.0),
                      upper_bound = std::vector<double>(n_constraint, 0.0);

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

bool Spline1dConstraint::AddThirdDerivativeSmoothConstraint() {
  if (x_knots_.size() <= 2) return false;

  size_t n_constraint = (x_knots_.size() - 2) * 4;
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(n_constraint, columns_);

  for (size_t i = 0; i < n_constraint; i += 4) {
    double left_coef = 1.0;
    double left_dcoef = 1.0;
    double left_ddcoef = 1.0;
    double left_dddcoef = 1.0;
    double right_coef = -1.0;
    double right_dcoef = -1.0;
    double right_ddcoef = -1.0;
    double right_dddcoef = -1.0;

    const double x_left = x_knots_[i / 4 + 1] - x_knots_[i / 4];
    const double x_right = 0.0;
    for (size_t j = 0; j < spline_param_num_; ++j) {
      constraint(i, spline_param_num_ * i / 4 + j) = left_coef;
      constraint(i, spline_param_num_ * (i / 4 + 1) + j) = right_coef;

      if (j >= 3) {
        constraint(i + 3, spline_param_num_ * i / 4 + j) =
            left_dddcoef * j * (j - 1) * (j - 2);
        constraint(i + 3, spline_param_num_ * (i / 4 + 1) + j) =
            right_dddcoef * j * (j - 1) * (j - 2);
        left_dddcoef = left_ddcoef;
        right_dddcoef = right_ddcoef;
      }

      if (j >= 2) {
        constraint(i + 2, spline_param_num_ * i / 4 + j) =
            left_ddcoef * j * (j - 1);
        constraint(i + 2, spline_param_num_ * (i / 4 + 1) + j) =
            right_ddcoef * j * (j - 1);
        left_ddcoef = left_dcoef;
        right_ddcoef = right_dcoef;
      }

      if (j >= 1) {
        constraint(i + 1, spline_param_num_ * i / 4 + j) = left_dcoef * j;
        constraint(i + 1, spline_param_num_ * (i / 4 + 1) + j) =
            right_dcoef * j;
        left_dcoef = left_coef;
        right_dcoef = right_coef;
      }
      left_coef *= x_left;
      right_coef *= x_right;
    }
  }

  std::vector<double> lower_bound = std::vector<double>(n_constraint, 0.0),
                      upper_bound = std::vector<double>(n_constraint, 0.0);

  return affine_constraint_.AddConstraint(constraint, lower_bound, upper_bound);
}

size_t Spline1dConstraint::FindSegStartIndex(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin(), x_knots_.end(), x);
  return std::min<size_t>(upper_bound - x_knots_.begin() - 1,
                          x_knots_.size() - 2);
}

const AffineConstraint& Spline1dConstraint::affine_constraint() const {
  return affine_constraint_;
}

const Eigen::MatrixXd& Spline1dConstraint::constraint_matrix() const {
  return affine_constraint_.constraint_matrix();
}

const std::vector<double>& Spline1dConstraint::lower_bound() const {
  return affine_constraint_.lower_bound();
}
const std::vector<double>& Spline1dConstraint::upper_bound() const {
  return affine_constraint_.upper_bound();
}

}  // namespace common
