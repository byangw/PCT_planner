#include "trajectory_optimization/gpmp_optimizer/factors_wnoa/gp_obstacle_factor.h"

gtsam::Vector GPObstacleFactorWnoa::evaluateError(
    const gtsam::Vector4& x1, boost::optional<gtsam::Matrix&> H1) const {
  double cost;
  Eigen::Vector2d grad;

  cost = map_->GetValueBilinearSafe(current_layer_, x1(0, 0), x1(2, 0),
                                    height_hint_, &grad);
  current_layer_ =
      map_->UpdateLayerSafe(current_layer_, x1(0, 0), x1(2, 0), height_hint_);
  height_hint_ =
      map_->GetHeightSafe(current_layer_, x1(0, 0), x1(2, 0), height_hint_);

  double error = 0.0;

  if (cost <= cost_threshold_) {
    if (H1) {
      *H1 = Eigen::MatrixXd::Zero(1, 4);
    }
  } else {
    error = (cost - cost_threshold_) * (cost - cost_threshold_);
    if (H1) {
      *H1 = Eigen::MatrixXd::Zero(1, 4);
      (*H1)(0, 0) = 2 * (cost - cost_threshold_) * grad(0);
      (*H1)(0, 2) = 2 * (cost - cost_threshold_) * grad(1);
    }
  }

  // if (verbose_) {
  // printKeys();
  // printf(
  //     "query pos (%f, %f), cost: %f, threshould: %f,  error: %f, grad: (%f, "
  //     "%f)\n",
  //     x1(0, 0), x1(2, 0), cost, cost_threshold_, error, grad(0), grad(1));
  // }

  return gtsam::Vector1(error);
}