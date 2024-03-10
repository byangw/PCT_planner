#include "trajectory_optimization/gpmp_optimizer/factors/gp_interpolate_obstacle_factor.h"

gtsam::Vector GPInterpolateObstacleFactor::evaluateError(
    const gtsam::Vector6& x1, const gtsam::Vector6& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  if (!initialized_) {
    int init_layer = current_layer_;
    gtsam::Vector6 x_tmp = gp_interpolator_.Interpolate(x1, x2);
    current_layer_ = map_->UpdateLayerSafe(current_layer_, x_tmp(0, 0),
                                           x_tmp(3, 0), height_hint_);
    height_hint_ = map_->GetHeightSafe(current_layer_, x_tmp(0, 0), x_tmp(3, 0),
                                       height_hint_);
    initialized_ = true;
  }

  double cost;
  Eigen::Vector2d grad;
  gtsam::Matrix66 J_x1, J_x2;
  gtsam::Matrix16 H = Eigen::MatrixXd::Zero(1, 6);
  gtsam::Vector6 x_inter = gp_interpolator_.Interpolate(x1, x2, &J_x1, &J_x2);

  cost = map_->GetValueBilinearSafe(current_layer_, x_inter(0, 0),
                                    x_inter(3, 0), height_hint_, &grad);
  current_layer_ = map_->UpdateLayerSafe(current_layer_, x_inter(0, 0),
                                         x_inter(3, 0), height_hint_);
  height_hint_ = map_->GetHeightSafe(current_layer_, x_inter(0, 0),
                                     x_inter(3, 0), height_hint_);
  double error = 0.0;

  if (cost > cost_threshold_) {
    error = (cost - cost_threshold_) * (cost - cost_threshold_);
    H(0, 0) = 2 * (cost - cost_threshold_) * grad(0);
    H(0, 3) = 2 * (cost - cost_threshold_) * grad(1);
  }

  if (H1 || H2) {
    *H1 = H * J_x1;
    *H2 = H * J_x2;
  }

  return gtsam::Vector1(error);
}