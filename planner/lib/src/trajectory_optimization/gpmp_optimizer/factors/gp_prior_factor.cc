#include "trajectory_optimization/gpmp_optimizer/factors/gp_prior_factor.h"

using gtsam::Matrix;

gtsam::Vector GPPriorFactor::evaluateError(
    const gtsam::Vector6& x1, const gtsam::Vector6& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  if (H1) *H1 = phi_;
  if (H2) *H2 = -gtsam::Matrix66::Identity();
  // printf("--------------------\n");
  // std::cout << "x1: " << x1.transpose() << std::endl;
  // std::cout << "x2: " << x2.transpose() << std::endl;
  // std::cout << (phi_ * x1 - x2).transpose() << std::endl;
  // std::cout << WhiteNoiseOnJerkModel2D::Q(0.1, delta_) << std::endl;
  // printf("--------------------\n");
  return phi_ * x1 - x2;
}
