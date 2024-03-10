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

#include <utility>
#include <vector>

#include "common/smoothing/spline1d_kernel_helper.h"

namespace common {

Spline1dKernelHelper& Spline1dKernelHelper::Instance() {
  static Spline1dKernelHelper spline1d_kernel_helper;
  return spline1d_kernel_helper;
}

Spline1dKernelHelper::Spline1dKernelHelper() { BuildKernelMap(); }

Eigen::MatrixXd Spline1dKernelHelper::Kernel(const uint32_t spline_order,
                                             const uint32_t nth_derivative,
                                             const double integral_length) {
  uint32_t num_params = spline_order + 1;

  std::string kernel_type =
      std::to_string(spline_order) + "_" + std::to_string(nth_derivative);
  const auto& derivative_matrix = kernel_map_[kernel_type];

  Eigen::MatrixXd term_matrix;
  IntegratedTermMatrix(num_params, nth_derivative, integral_length,
                       &term_matrix);

  return derivative_matrix.block(0, 0, num_params, num_params)
      .cwiseProduct(term_matrix);
}

void Spline1dKernelHelper::BuildKernelMap() {
  for (uint32_t order = 3; order <= 5; ++order) {
    const uint32_t num_params = order + 1;
    for (uint32_t derivative = 0; derivative <= 3; ++derivative) {
      std::string kernel_type =
          std::to_string(order) + "_" + std::to_string(derivative);
      if (derivative == 0) {
        CalculateFx(num_params);
        kernel_map_[kernel_type] = kernel_fx_;
      } else if (derivative == 1) {
        CalculateDerivative(num_params);
        kernel_map_[kernel_type] = kernel_derivative_;
      } else if (derivative == 2) {
        CalculateSecondOrderDerivative(num_params);
        kernel_map_[kernel_type] = kernel_second_order_derivative_;
      } else if (derivative == 3) {
        CalculateThirdOrderDerivative(num_params);
        kernel_map_[kernel_type] = kernel_third_order_derivative_;
      }
    }
  }
}

void Spline1dKernelHelper::IntegratedTermMatrix(
    const uint32_t num_params, const uint16_t derivative, const double x,
    Eigen::MatrixXd* term_matrix) const {
  if (term_matrix->rows() != term_matrix->cols() ||
      term_matrix->rows() != static_cast<int>(num_params)) {
    term_matrix->resize(num_params, num_params);
  }

  std::vector<double> x_pow(2 * num_params + 1, 1.0);
  for (uint32_t i = 1; i < 2 * num_params + 1; ++i) {
    x_pow[i] = x_pow[i - 1] * x;
  }

  if (derivative == 0) {
    for (uint32_t r = 0; r < num_params; ++r) {
      for (uint32_t c = 0; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c + 1];
      }
    }

  } else if (derivative == 1) {
    for (uint32_t r = 1; r < num_params; ++r) {
      for (uint32_t c = 1; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 1];
      }
    }
    (*term_matrix).block(0, 0, num_params, 1) =
        Eigen::MatrixXd::Zero(num_params, 1);
    (*term_matrix).block(0, 0, 1, num_params) =
        Eigen::MatrixXd::Zero(1, num_params);

  } else if (derivative == 2) {
    for (uint32_t r = 2; r < num_params; ++r) {
      for (uint32_t c = 2; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 3];
      }
    }
    (*term_matrix).block(0, 0, num_params, 2) =
        Eigen::MatrixXd::Zero(num_params, 2);
    (*term_matrix).block(0, 0, 2, num_params) =
        Eigen::MatrixXd::Zero(2, num_params);

  } else if (derivative == 3) {
    for (uint32_t r = 3; r < num_params; ++r) {
      for (uint32_t c = 3; c < num_params; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 5];
      }
    }
    (*term_matrix).block(0, 0, num_params, 3) =
        Eigen::MatrixXd::Zero(num_params, 3);
    (*term_matrix).block(0, 0, 3, num_params) =
        Eigen::MatrixXd::Zero(3, num_params);
  }
}

void Spline1dKernelHelper::CalculateFx(const uint32_t num_params) {
  kernel_fx_ = Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 0; r < kernel_fx_.rows(); ++r) {
    for (int c = 0; c < kernel_fx_.cols(); ++c) {
      kernel_fx_(r, c) = 1.0 / (r + c + 1.0);
    }
  }
}

void Spline1dKernelHelper::CalculateDerivative(const uint32_t num_params) {
  kernel_derivative_ = Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 1; r < kernel_derivative_.rows(); ++r) {
    for (int c = 1; c < kernel_derivative_.cols(); ++c) {
      kernel_derivative_(r, c) = r * c / (r + c - 1.0);
    }
  }
}

void Spline1dKernelHelper::CalculateSecondOrderDerivative(
    const uint32_t num_params) {
  kernel_second_order_derivative_ =
      Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 2; r < kernel_second_order_derivative_.rows(); ++r) {
    for (int c = 2; c < kernel_second_order_derivative_.cols(); ++c) {
      kernel_second_order_derivative_(r, c) =
          (r * r - r) * (c * c - c) / (r + c - 3.0);
    }
  }
}

void Spline1dKernelHelper::CalculateThirdOrderDerivative(
    const uint32_t num_params) {
  kernel_third_order_derivative_ =
      Eigen::MatrixXd::Zero(num_params, num_params);
  for (int r = 3; r < kernel_third_order_derivative_.rows(); ++r) {
    for (int c = 3; c < kernel_third_order_derivative_.cols(); ++c) {
      kernel_third_order_derivative_(r, c) =
          (r * r - r) * (r - 2) * (c * c - c) * (c - 2) / (r + c - 5.0);
    }
  }
}
}  // namespace common
