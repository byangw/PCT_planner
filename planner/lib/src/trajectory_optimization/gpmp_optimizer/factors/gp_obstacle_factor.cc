#include "trajectory_optimization/gpmp_optimizer/factors/gp_obstacle_factor.h"

gtsam::Vector GPObstacleFactor::evaluateError(
    const gtsam::Vector6& x1, boost::optional<gtsam::Matrix&> H1) const {
  double cost;
  Eigen::Vector2d grad;

  cost = map_->GetValueBilinearSafe(current_layer_, x1(0, 0), x1(3, 0),
                                    height_hint_, &grad);
  current_layer_ =
      map_->UpdateLayerSafe(current_layer_, x1(0, 0), x1(3, 0), height_hint_);
  height_hint_ =
      map_->GetHeightSafe(current_layer_, x1(0, 0), x1(3, 0), height_hint_);

  double error = 0.0;

  if (cost <= cost_threshold_) {
    if (H1) {
      *H1 = Eigen::MatrixXd::Zero(1, 6);
    }
  } else {
    error = (cost - cost_threshold_) * (cost - cost_threshold_);
    if (H1) {
      *H1 = Eigen::MatrixXd::Zero(1, 6);
      (*H1)(0, 0) = 2 * (cost - cost_threshold_) * grad(0);
      (*H1)(0, 3) = 2 * (cost - cost_threshold_) * grad(1);
    }
  }

  return gtsam::Vector1(error);
}