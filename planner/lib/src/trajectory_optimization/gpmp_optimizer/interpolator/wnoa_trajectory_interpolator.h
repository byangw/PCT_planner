#pragma once

#include "gtsam/nonlinear/Values.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoa_interpolator.hpp"

class WnoaTrajectoryInterpolator {
 public:
  WnoaTrajectoryInterpolator() = default;
  ~WnoaTrajectoryInterpolator() = default;

  WnoaTrajectoryInterpolator(const Eigen::MatrixXd& nodes, const double dt,
                             const double qc);

  Eigen::MatrixXd GenerateTrajectory(const int sub_sample_num) const;

 private:
  double qc_ = 0.0;
  double dt_ = 0.0;
  int num_nodes_ = 0;
  GPInterpolatorWnoa interpolator_;
  Eigen::MatrixXd nodes_;
};
