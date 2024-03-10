#include "ele_planner/offline_ele_planner.h"

void OfflineElePlanner::InitMap(
    const double a_start_cost_threshold, const double safe_cost_margin,
    const double resolution, const int num_layers, const double step_cost_weight,
    const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& height_map,
    const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
    const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y) {
  path_finder_.Init(a_start_cost_threshold, num_layers, resolution, step_cost_weight, cost_map,
                    height_map, ele_map);
  map_ = std::make_shared<DenseElevationMap>();
  map_->Init(resolution, num_layers, cost_map, ele_map, height_map, ceiling,
             grad_x, grad_y);
  trajectory_optimizer_ = GPMPOptimizerWnoa(safe_cost_margin, map_);
  trajectory_optimizer_wnoj_ =
      GPMPOptimizer(safe_cost_margin, max_heading_rate_, map_);
}

bool OfflineElePlanner::Plan(const Eigen::Vector3i& start,
                             const Eigen::Vector3i& goal, const bool optimize) {
  if (!path_finder_.Search(start, goal)) {
    printf("A star Failed!\n");
    return false;
  }

  if (optimize) {
    path_ = path_finder_.GetPathPoints();
    path_.front().ref_v = 1;
    path_.back().ref_v = 1;

    bool success = false;
    if (use_quintic_) {
      success = trajectory_optimizer_wnoj_.GenerateTrajectory(path_, 200);
    } else {
      success = trajectory_optimizer_.GenerateTrajectory(path_, 200);
    }

    return success;
  }

  return true;
}
