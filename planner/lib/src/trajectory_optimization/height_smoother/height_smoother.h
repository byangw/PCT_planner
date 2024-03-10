#pragma once
#include <Eigen/Dense>

class HeightSmoother {
 public:
  HeightSmoother() = default;
  ~HeightSmoother() = default;

  Eigen::VectorXd Smooth(const Eigen::VectorXd& coarse_height,
                         const Eigen::VectorXd& upper_bound, const double dt,
                         const int N, const double knot_interval);
};