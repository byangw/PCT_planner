#pragma once

#include <Eigen/Dense>
#include <memory>

#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/height_smoother/height_smoother.h"

class GPMPOptimizer {
 public:
  GPMPOptimizer() = default;
  ~GPMPOptimizer() = default;

  GPMPOptimizer(const double safe_cost_margin, const double max_heading_rate,
                std::shared_ptr<DenseElevationMap> map)
      : map_(map),
        safe_cost_margin_(safe_cost_margin),
        max_heading_rate_(max_heading_rate) {}

  bool GenerateTrajectory(const std::vector<PathPoint>& path, const double T);

  void set_max_iterations(int max_iterations) {
    max_iterations_ = max_iterations;
  }

  void SetReferenceHeight(const double height) { reference_height_ = height; }

  Eigen::MatrixXd GetOptInitValue() const { return opt_init_value_; }
  Eigen::MatrixXd GetOptInitLayer() const { return opt_init_layer_; }
  Eigen::MatrixXd GetResultMatrix() const { return trajectory_; }
  Eigen::VectorXd GetResultLayers() const { return opt_layers_; }
  Eigen::VectorXd GetResultHeight() const { return opt_height_; }
  Eigen::VectorXd GetResultCeiling() const { return opt_ceiling_; }
  Eigen::VectorXd GetHeadingRate() const;

  void set_sample_interval(const int sample_interval) {
    sample_interval_ = sample_interval;
  }

  void SetDebug(const bool flag) { debug_ = flag; }

 protected:
  void PathPointToNode(const PathPoint& path_point, Vector6& x);

  void SubSamplePath(const std::vector<PathPoint>& path,
                     std::vector<PathPoint>& sub_sampled_path);

 private:
  bool debug_ = false;

  std::shared_ptr<DenseElevationMap> map_ = nullptr;
  Eigen::MatrixXd opt_init_value_;
  Eigen::VectorXd opt_init_layer_;
  Eigen::MatrixXd opt_results_;
  Eigen::VectorXd opt_layers_;
  Eigen::VectorXd opt_height_;
  Eigen::VectorXd opt_ceiling_;
  Eigen::MatrixXd trajectory_;

  int sample_interval_ = 10;
  int interpolate_num_ = 8;
  double safe_cost_margin_ = 10;
  int max_iterations_ = 100;
  double max_heading_rate_ = 0.5;
  double reference_height_ = 0.1;

  HeightSmoother height_smoother_;
};
