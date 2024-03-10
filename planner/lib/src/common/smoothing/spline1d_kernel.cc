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

#include "common/smoothing/spline1d_kernel.h"

//#include <glog/logging.h>

#include "common/smoothing/spline1d_kernel_helper.h"

namespace common {
Spline1dKernel::Spline1dKernel(const std::vector<double>& x_knots,
                               const uint32_t spline_order)
    : x_knots_(x_knots), spline_order_(spline_order) {
  spline_param_num_ = spline_order_ + 1;
  total_params_ = (x_knots_.size() - 1) * spline_param_num_;

  kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
  gradient_ = Eigen::MatrixXd::Zero(total_params_, 1);
}

const Eigen::MatrixXd& Spline1dKernel::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& Spline1dKernel::gradient() const { return gradient_; }

void Spline1dKernel::AddRegularization(const double regularized_param) {
  Eigen::MatrixXd id_matrix =
      Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols());
  kernel_matrix_ += 2.0 * id_matrix * regularized_param;
}

void Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel) {
  //CHECK_EQ(kernel.rows() == kernel_matrix_.rows())
      // << "kernel row size doesn't match";
  //CHECK_EQ(kernel.cols() == kernel_matrix_.cols())
      // << "kernel col size doesn't match";

  kernel_matrix_ += kernel;
}

void Spline1dKernel::AddGradient(const Eigen::MatrixXd& gradient) {
  //CHECK_EQ(gradient.rows() == gradient_.rows())
      // << "gradient row size doesn't match";
  //CHECK_EQ(gradient.cols() == gradient_.cols())
      // << "gradient col size doesn't match";

  gradient_ += gradient;
}

void Spline1dKernel::AddNthDerivativekernelMatrix(const uint32_t n,
                                                  const double weight) {
  uint32_t spline_seg_num = x_knots_.size() - 1;
  for (uint32_t i = 0; i < spline_seg_num; ++i) {
    Eigen::MatrixXd seg_kernel_matrix =
        2 *
        Spline1dKernelHelper::Instance().Kernel(spline_order_, n,
                                                x_knots_[i + 1] - x_knots_[i]) *
        weight;
    kernel_matrix_.block(i * spline_param_num_, i * spline_param_num_,
                         spline_param_num_, spline_param_num_) +=
        seg_kernel_matrix;
  }
}

// 计算sum_{t==0}^{x} {f'(t) * f'(t)}, 用以最小化导数平方
void Spline1dKernel::AddDerivativeKernelMatrix(const double weight) {
  AddNthDerivativekernelMatrix(1, weight);
}

void Spline1dKernel::AddSecondOrderDerivativeMatrix(const double weight) {
  AddNthDerivativekernelMatrix(2, weight);
}

void Spline1dKernel::AddThirdOrderDerivativeMatrix(const double weight) {
  AddNthDerivativekernelMatrix(3, weight);
}

/*
  计算[spline(t) - ref_fx]的最小二次型
*/
bool Spline1dKernel::AddReferenceLineKernelMatrix(
    const std::vector<double>& x_coord, const std::vector<double>& ref_fx,
    const double weight) {
  if (ref_fx.size() != x_coord.size()) {
    return false;
  }

  const uint32_t num_params = spline_order_ + 1;
  for (size_t i = 0; i < x_coord.size(); ++i) {
    auto cur_index = FindSegStartIndex(x_coord[i]);
    double cur_rel_x = x_coord[i] - x_knots_[cur_index];
    // update offset
    double offset_coef = -2.0 * ref_fx[i] * weight;
    for (size_t j = 0; j < num_params; ++j) {
      gradient_(j + cur_index * num_params, 0) += offset_coef;
      offset_coef *= cur_rel_x;
    }
    // update kernel matrix
    Eigen::MatrixXd ref_kernel(num_params, num_params);

    double cur_x = 1.0;
    std::vector<double> power_x;
    for (uint32_t n = 0; n + 1 < 2 * num_params; ++n) {
      power_x.emplace_back(cur_x);
      cur_x *= cur_rel_x;
    }

    for (uint32_t r = 0; r < num_params; ++r) {
      for (uint32_t c = 0; c < num_params; ++c) {
        ref_kernel(r, c) = 2.0 * power_x[r + c];
      }
    }

    kernel_matrix_.block(cur_index * num_params, cur_index * num_params,
                         num_params, num_params) += weight * ref_kernel;
  }
  return true;
}

bool Spline1dKernel::AddDerivativeReferenceLineKernelMatrix(
    const std::vector<double>& x_coord, const std::vector<double>& ref_dfx,
    const double weight) {
  if (ref_dfx.size() != x_coord.size()) {
    return false;
  }

  const uint32_t num_params = spline_order_ + 1;
  for (size_t i = 0; i < x_coord.size(); ++i) {
    auto cur_index = FindSegStartIndex(x_coord[i]);
    double cur_rel_x = x_coord[i] - x_knots_[cur_index];
    // update offset
    double x_power = 1.0;
    Eigen::VectorXd coefficients = Eigen::VectorXd::Zero(6);
    for (int j = 1; j < 6; ++j) {
      coefficients(j) = j * x_power;
      x_power *= cur_rel_x;
    }

    for (size_t j = 0; j < num_params; ++j) {
      gradient_(j + cur_index * num_params, 0) -=
          2.0 * ref_dfx[i] * weight * coefficients(j);
    }
    // LOG(INFO) << "ref: " << ref_dfx[i];
    // update kernel matrix
    Eigen::MatrixXd ref_kernel = 2 * coefficients * coefficients.transpose();

    kernel_matrix_.block(cur_index * num_params, cur_index * num_params,
                         num_params, num_params) += weight * ref_kernel;
  }
  return true;
}

size_t Spline1dKernel::FindSegStartIndex(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin(), x_knots_.end(), x);
  return std::min<size_t>(upper_bound - x_knots_.begin() - 1,
                          x_knots_.size() - 2);
}
}  // namespace common
