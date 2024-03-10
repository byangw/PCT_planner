#pragma once

#include "gtsam/nonlinear/Values.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_interpolator.hpp"

class WnojTrajectoryInterpolator {
 public:
  WnojTrajectoryInterpolator() = default;
  ~WnojTrajectoryInterpolator() = default;

  WnojTrajectoryInterpolator(const Eigen::MatrixXd& nodes, const double dt,
                             const double qc);

  Eigen::MatrixXd GenerateTrajectory(const int sub_sample_num) const;

 private:
  double qc_ = 0.0;
  double dt_ = 0.0;
  int num_nodes_ = 0;
  GPInterpolator interpolator_;
  Eigen::MatrixXd nodes_;
};
