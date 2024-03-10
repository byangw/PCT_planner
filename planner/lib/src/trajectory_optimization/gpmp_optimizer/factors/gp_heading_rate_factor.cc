#include "trajectory_optimization/gpmp_optimizer/factors/gp_heading_rate_factor.h"

gtsam::Vector GPHeadingRateFactor::evaluateError(
    const gtsam::Vector6& x1, boost::optional<gtsam::Matrix&> H1) const {
  double dx = x1(1);
  double ddx = x1(2);
  double dy = x1(4);
  double ddy = x1(5);
  double sqr_dx_dy = (dx * dx + dy * dy + 1e-6);
  double ddy_dx_dy_ddx = ddy * dx - dy * ddx;
  double dot_theta = ddy_dx_dy_ddx / sqr_dx_dy;

  double error = 0.0;
  Eigen::MatrixXd J_dot_theta = Eigen::MatrixXd::Zero(1, 6);

  if (abs(dot_theta) > max_heading_rate_) {
    if (H1) {
      J_dot_theta(0, 1) = (ddy * dy * dy - ddy * dx * dx + 2 * dx * dy * ddx) /
                          (sqr_dx_dy * sqr_dx_dy);
      J_dot_theta(0, 2) = -dy / sqr_dx_dy;
      J_dot_theta(0, 4) = (-ddx * dx * dx + ddx * dy * dy - 2 * dx * dy * ddy) /
                          (sqr_dx_dy * sqr_dx_dy);
      J_dot_theta(0, 5) = dx / sqr_dx_dy;
      *H1 = 2 * (dot_theta - max_heading_rate_) * J_dot_theta;
    }
  }

  // if (dot_theta > max_heading_rate_) {
  //   error = (dot_theta - max_heading_rate_) * (dot_theta -
  //   max_heading_rate_); if (H1) {
  //     *H1 = 2 * (dot_theta - max_heading_rate_) * J_dot_theta;
  //   }
  // } else if (dot_theta < -max_heading_rate_) {
  //   error = (dot_theta + max_heading_rate_) * (dot_theta +
  //   max_heading_rate_); if (H1) {
  //     *H1 = 2 * (dot_theta + max_heading_rate_) * J_dot_theta;
  //   }
  // } else {
  //   if (H1) {
  //     *H1 = Eigen::MatrixXd::Zero(1, 6);
  //   }
  // }

  if (dot_theta > max_heading_rate_) {
    error = dot_theta - max_heading_rate_;
    if (H1) {
      *H1 = J_dot_theta;
    }
  } else if (dot_theta < -max_heading_rate_) {
    error = -max_heading_rate_ - dot_theta;
    if (H1) {
      *H1 = -J_dot_theta;
    }
  } else {
    if (H1) {
      *H1 = Eigen::MatrixXd::Zero(1, 6);
    }
  }

  return gtsam::Vector1(error);
}