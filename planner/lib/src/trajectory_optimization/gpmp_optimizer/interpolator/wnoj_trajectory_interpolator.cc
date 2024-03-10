
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.h"

#include "Eigen/Core"

using Vector6 = Eigen::Matrix<double, 6, 1>;

WnojTrajectoryInterpolator::WnojTrajectoryInterpolator(
    const Eigen::MatrixXd& nodes, const double dt, const double qc)
    : nodes_(nodes), interpolator_(qc, dt, 0.0), dt_(dt), qc_(qc) {
  num_nodes_ = nodes.rows();
}

Eigen::MatrixXd WnojTrajectoryInterpolator::GenerateTrajectory(
    const int sub_sample_num) const {
  double sample_dt = dt_ / (sub_sample_num + 1);
  int num_points = (num_nodes_ - 1) * (sub_sample_num) + num_nodes_;

  Eigen::MatrixXd trajectory(num_points, 6);

  int index = 0;
  for (int i = 1; i < num_nodes_; i++) {
    trajectory.row(index) = nodes_.row(i - 1);
    index++;
    Vector6 x1 = nodes_.row(i - 1).transpose();
    Vector6 x2 = nodes_.row(i).transpose();
    for (int j = 0; j < sub_sample_num; j++) {
      double tau = (j + 1) * sample_dt;
      Vector6 x_inter = interpolator_.Interpolate(x1, x2, qc_, dt_, tau);
      trajectory.row(index) = x_inter.transpose();
      index++;
    }
  }
  trajectory.row(num_points - 1) = nodes_.row(num_nodes_ - 1);
  index++;
  assert (index == num_points);

  return trajectory;
}