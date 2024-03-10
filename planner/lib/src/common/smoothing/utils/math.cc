/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "common/smoothing/utils/math.h"

#include <cmath>
#include <random>

namespace common {

double NormalizeAngle(const double angle) {
  constexpr double k2Pi = 2 * M_PI;
  double a = std::fmod(angle + M_PI, k2Pi);
  if (a < 0.0) {
    a += k2Pi;
  }
  return a - M_PI;
}

double InterpolateAngle(const double a0, const double t0, const double a1,
                        const double t1, const double t) {
  if (std::abs(t1 - t0) <= 1e-6) {
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

int RandomInt(const int size) {
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 gen(rd());  // seed the generator
  std::uniform_int_distribution<> distr(0, size - 1);  // define the range
  return distr(gen);
}

double RandomDouble(const double lb, const double ub) {
  std::uniform_real_distribution<double> distr(lb, ub);
  std::default_random_engine re(std::random_device{}());
  return distr(re);
}

double Curvature(const double dx, const double d2x, const double dy,
                 const double d2y) {
  // kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
  const double a = dx * d2y - dy * d2x;
  auto norm_square = dx * dx + dy * dy;
  auto norm = std::sqrt(norm_square);
  const double b = norm * norm_square;
  return a / b;
}

double CurvatureDerivative(const double dx, const double d2x, const double d3x,
                           const double dy, const double d2y,
                           const double d3y) {
  const double a = dx * d2y - dy * d2x;
  const double b = dx * d3y - dy * d3x;
  const double c = dx * d2x + dy * d2y;
  const double d = dx * dx + dy * dy;

  return (b * d - 3.0 * a * c) / (d * d * d);
}

}  // namespace common
