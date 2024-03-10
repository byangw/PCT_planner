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

#include "common/smoothing/spline2d_kernel.h"

//#include <glog/logging.h>

namespace common {

Spline2dKernel::Spline2dKernel(const std::vector<double>& t_knots,
                               const uint32_t spline_order)
    : t_knots_(t_knots), spline_order_(spline_order) {
  x_kernel_ = Spline1dKernel(t_knots_, spline_order_);
  y_kernel_ = Spline1dKernel(t_knots_, spline_order_);

  spline_param_num_ = spline_order_ + 1;
  total_params_ = 2 * (t_knots_.size() - 1) * spline_param_num_;

  cooperative_kernel_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
}

void Spline2dKernel::AddRegularization(const double regularized_param) {
  x_kernel_.AddRegularization(regularized_param);
  y_kernel_.AddRegularization(regularized_param);
}

void Spline2dKernel::Add2dDerivativeKernelMatrix(const double weight) {
  x_kernel_.AddDerivativeKernelMatrix(weight);
  y_kernel_.AddDerivativeKernelMatrix(weight);
}

void Spline2dKernel::Add2dSecondOrderDerivativeMatrix(const double weight) {
  x_kernel_.AddSecondOrderDerivativeMatrix(weight);
  y_kernel_.AddSecondOrderDerivativeMatrix(weight);
}

void Spline2dKernel::Add2dThirdOrderDerivativeMatrix(const double weight) {
  x_kernel_.AddThirdOrderDerivativeMatrix(weight);
  y_kernel_.AddThirdOrderDerivativeMatrix(weight);
}

bool Spline2dKernel::Add2dReferenceLineKernelMatrix(
    const std::vector<double>& t_coord,
    const vector_Eigen<Eigen::Vector2d>& ref_ft, const double weight) {
  size_t ref_size = ref_ft.size();
  std::vector<double> fx(ref_size), fy(ref_size);
  for (size_t i = 0; i < ref_size; ++i) {
    fx[i] = ref_ft[i].x();
    fy[i] = ref_ft[i].y();
  }

  bool add_x_ref = x_kernel_.AddReferenceLineKernelMatrix(t_coord, fx, weight);
  bool add_y_ref = y_kernel_.AddReferenceLineKernelMatrix(t_coord, fy, weight);

  return add_x_ref && add_y_ref;
}

void Spline2dKernel::Add2dLateralOffsetKernelMatrix(
    const std::vector<double>& t_coord,
    const vector_Eigen<Eigen::Vector2d>& ref_ft,
    const ::std::vector<double> ref_angle, const double weight) {
  //CHECK_EQ(t_coord.size(), ref_ft.size());
  //CHECK_EQ(ref_angle.size(), ref_ft.size());

  const uint32_t num_params = spline_order_ + 1;

  Eigen::MatrixXd kernel_x =
      Eigen::MatrixXd::Zero(total_params_ / 2, total_params_ / 2);
  Eigen::MatrixXd kernel_y =
      Eigen::MatrixXd::Zero(total_params_ / 2, total_params_ / 2);
  Eigen::MatrixXd gradient_x = Eigen::MatrixXd::Zero(total_params_ / 2, 1);
  Eigen::MatrixXd gradient_y = Eigen::MatrixXd::Zero(total_params_ / 2, 1);

  Eigen::MatrixXd kernel_x_seg, kernel_y_seg;
  Eigen::MatrixXd gradient_x_seg, gradient_y_seg;

  for (size_t i = 0; i < t_coord.size(); ++i) {
    const uint32_t index = FindSegStartIndex(t_coord[i]);
    const double t_corrected = t_coord[i] - t_knots_[index];

    const double sin_0 = std::sin(ref_angle[i]);
    const double cos_0 = std::cos(ref_angle[i]);
    const double sqr_sin_0 = sin_0 * sin_0;
    const double sqr_cos_0 = cos_0 * cos_0;

    kernel_x_seg = Eigen::MatrixXd::Zero(num_params, num_params);
    kernel_y_seg = Eigen::MatrixXd::Zero(num_params, num_params);
    gradient_x_seg = Eigen::MatrixXd::Zero(num_params, 1);
    gradient_y_seg = Eigen::MatrixXd::Zero(num_params, 1);

    std::vector<double> t_power;
    double t_current = 1.0;
    for (uint32_t n = 0; n + 1 < 2 * num_params; ++n) {
      t_power.emplace_back(t_current);
      t_current *= t_corrected;
    }

    /**
     * independent part:
     * sin0^2 * x^2 - 2 * (xr * sin0^2 - yr * sin0 * cos0) * x
     * cos0^2 * y^2 - 2 * (yr * cos0^2 - xr * sin0 * cos0) * y
     */
    const double gradient_x_coeff =
        -2 * ref_ft[i].x() * sqr_sin_0 + 2 * ref_ft[i].y() * sin_0 * cos_0;
    const double gradient_y_coeff =
        -2 * ref_ft[i].y() * sqr_cos_0 + 2 * ref_ft[i].x() * sin_0 * cos_0;

    for (uint32_t r = 0; r < num_params; ++r) {
      gradient_x_seg(r, 0) = gradient_x_coeff * t_power[r];
      gradient_y_seg(r, 0) = gradient_y_coeff * t_power[r];
      for (uint32_t c = 0; c < num_params; ++c) {
        kernel_x_seg(r, c) = 2.0 * sqr_sin_0 * t_power[r + c];
        kernel_y_seg(r, c) = 2.0 * sqr_cos_0 * t_power[r + c];
      }
    }

    kernel_x.block(index * num_params, index * num_params, num_params,
                   num_params) += weight * kernel_x_seg;
    kernel_y.block(index * num_params, index * num_params, num_params,
                   num_params) += weight * kernel_y_seg;
    gradient_x.block(index * num_params, 0, num_params, 1) +=
        weight * gradient_x_seg;
    gradient_y.block(index * num_params, 0, num_params, 1) +=
        weight * gradient_y_seg;

    /* cooperative part: -2*x*y*sin0*cos0 */
    const uint32_t x_offset = index * num_params;
    const uint32_t y_offset = total_params_ / 2 + index * num_params;
    const double cooperative_coeff = weight * sin_0 * cos_0;

    for (uint32_t r = 0; r < num_params; ++r) {
      for (uint32_t c = 0; c < num_params; ++c) {
        cooperative_kernel_(x_offset + r, y_offset + c) +=
            -2 * cooperative_coeff * t_power[r + c];
        cooperative_kernel_(y_offset + r, x_offset + c) +=
            -2 * cooperative_coeff * t_power[r + c];
      }
    }
  }

  x_kernel_.AddKernel(kernel_x);
  x_kernel_.AddGradient(gradient_x);
  y_kernel_.AddKernel(kernel_y);
  y_kernel_.AddGradient(gradient_y);
}

void Spline2dKernel::Add2dLongitudinalOffsetKernelMatrix(
    const std::vector<double>& t_coord,
    const vector_Eigen<Eigen::Vector2d>& ref_ft,
    const ::std::vector<double> ref_angle, const double weight) {
  std::vector<double> ref_angle_longitudinal;
  for (size_t i = 0; i < ref_angle.size(); ++i) {
    ref_angle_longitudinal.emplace_back(ref_angle[i] - M_PI_2);
  }
  Add2dLateralOffsetKernelMatrix(t_coord, ref_ft, ref_angle_longitudinal,
                                 weight);
}

const Eigen::MatrixXd& Spline2dKernel::kernel_matrix() {
  uint32_t x_rows = x_kernel_.kernel_matrix().rows();
  uint32_t y_rows = y_kernel_.kernel_matrix().rows();

  kernel_matrix_ = Eigen::MatrixXd::Zero(x_rows + y_rows, total_params_);

  kernel_matrix_.block(0, 0, x_rows, total_params_ / 2) =
      x_kernel_.kernel_matrix();
  kernel_matrix_.block(x_rows, total_params_ / 2, y_rows, total_params_ / 2) =
      y_kernel_.kernel_matrix();

  kernel_matrix_ += cooperative_kernel_;

  return kernel_matrix_;
}

const Eigen::MatrixXd& Spline2dKernel::gradient() {
  uint32_t x_rows = x_kernel_.gradient().rows();
  uint32_t y_rows = y_kernel_.gradient().rows();

  gradient_ = Eigen::MatrixXd::Zero(x_rows + y_rows, 1);

  gradient_.block(0, 0, x_rows, 1) = x_kernel_.gradient();
  gradient_.block(x_rows, 0, y_rows, 1) = y_kernel_.gradient();

  return gradient_;
}

size_t Spline2dKernel::FindSegStartIndex(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin(), t_knots_.end(), t);
  return std::min<size_t>(upper_bound - t_knots_.begin() - 1,
                          t_knots_.size() - 2);
}
}  // namespace common
