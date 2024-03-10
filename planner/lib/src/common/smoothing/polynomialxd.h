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

#include <cinttypes>
#include <vector>

namespace common {

/**
 * @class PolynomialXd
 * @brief y = a0 + a1*x + a2*x2^2 + ... + an*xn^n
 */
class PolynomialXd {
 public:
  PolynomialXd() = default;
  explicit PolynomialXd(const std::uint32_t order);
  explicit PolynomialXd(const std::vector<double>& params);

  ~PolynomialXd() = default;

  double operator()(const double x) const;
  double operator[](const std::uint32_t index) const;

  void SetParams(const std::vector<double>& params);

  static PolynomialXd DerivedFrom(const PolynomialXd& base);
  static PolynomialXd IntegratedFrom(const PolynomialXd& base,
                                     const double intercept = 0.0);

  std::uint32_t order() const;

  const std::vector<double>& params() const;

 private:
  std::vector<double> params_;
};
}  // namespace common
