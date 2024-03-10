#include "trajectory_optimization/gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.h"

gtsam::Vector GPInterpolateHeadingRateFactor::evaluateError(
    const gtsam::Vector6& x1, const gtsam::Vector6& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  double cost;
  gtsam::Matrix66 J_x1, J_x2;
  gtsam::Matrix16 H = Eigen::MatrixXd::Zero(1, 6);
  gtsam::Vector6 x_inter = gp_interpolator_.Interpolate(x1, x2, &J_x1, &J_x2);

  double dx = x_inter(1);
  double ddx = x_inter(2);
  double dy = x_inter(4);
  double ddy = x_inter(5);
  double sqr_dx_dy = (dx * dx + dy * dy + 1e-6);
  double ddy_dx_dy_ddx = ddy * dx - dy * ddx;
  double dot_theta = ddy_dx_dy_ddx / sqr_dx_dy;

  double error = 0.0;
  Eigen::MatrixXd J_dot_theta = Eigen::MatrixXd::Zero(1, 6);

  if (abs(dot_theta) > max_heading_rate_) {
    if (H1 || H2) {
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
  //   max_heading_rate_); if (H1 || H2) {
  //     H = 2 * (dot_theta - max_heading_rate_) * J_dot_theta;
  //   }
  // } else if (dot_theta < -max_heading_rate_) {
  //   error = (dot_theta + max_heading_rate_) * (dot_theta +
  //   max_heading_rate_); if (H1 || H2) {
  //     H = 2 * (dot_theta + max_heading_rate_) * J_dot_theta;
  //   }
  // }

  if (dot_theta > max_heading_rate_) {
    error = dot_theta - max_heading_rate_;
    if (H1 || H2) {
      H = J_dot_theta;
    }
  } else if (dot_theta < -max_heading_rate_) {
    error = -max_heading_rate_ - dot_theta;
    if (H1 || H2) {
      H = -J_dot_theta;
    }
  }

  // if (abs(error) > 0) {
  //   printf("error: %f, heading rate: %f, max heading rate: %f, pos(%f, %f)\n",
  //          error, dot_theta, max_heading_rate_, x_inter(0), x_inter(3));
  // }

  if (H1 || H2) {
    *H1 = H * J_x1;
    *H2 = H * J_x2;
  }

  return gtsam::Vector1(error);
}