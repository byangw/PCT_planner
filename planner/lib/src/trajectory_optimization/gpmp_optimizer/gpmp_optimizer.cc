#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"

#include <chrono>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/Symbol.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_heading_rate_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_interpolate_obstacle_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_obstacle_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_prior_factor.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.h"

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using PriorFactor6 = gtsam::PriorFactor<Vector6>;

constexpr double kQc = 0.1 * 0.1;
constexpr double kQcHeading = 0.01;

bool GPMPOptimizer::GenerateTrajectory(const std::vector<PathPoint>& input_path,
                                       const double T) {
  auto t0 = std::chrono::high_resolution_clock::now();

  Vector6 s_init, s_target;
  s_init << 0.001, 0.1, 1, 0.001, 0.1, 1;
  s_target << 0.001, 1, 1, 0.001, 1, 1;

  static auto sigma_initial = Diagonal::Sigmas(s_init);
  static auto sigma_goal = Diagonal::Sigmas(s_target);

  std::vector<PathPoint> path;
  SubSamplePath(input_path, path);

  double T_tmp = static_cast<double>(input_path.size() - 1) / 3;
  int N = path.size();
  double dt = T_tmp / (N - 1);
  double tau = dt / (interpolate_num_ + 1);
  std::cout<<"tau: "<<tau<<std::endl;

  Vector6 x0, xN;
  PathPointToNode(path.front(), x0);
  PathPointToNode(path.back(), xN);

  std::cout << "x0: " << x0.transpose() << std::endl;
  std::cout << "xN: " << xN.transpose() << std::endl;

  auto graph = gtsam::NonlinearFactorGraph();
  int factor_idx = 0;
  std::vector<int> obstacle_factor_idx;

  gtsam::Values init_values;
  opt_init_value_ = Eigen::MatrixXd(6, N + (N - 1) * interpolate_num_);
  opt_init_layer_ = Eigen::VectorXd(N + (N - 1) * interpolate_num_);

  int col_index = 0;
  for (int i = 0; i < N; ++i) {
    Vector6 x1, x2;
    PathPointToNode(path[i], x1);
    init_values.insert<Vector6>(gtsam::Symbol('x', i), x1);
    opt_init_value_.col(col_index) = x1;
    opt_init_layer_(col_index) = path[i].layer;
    if (debug_) {
      printf("path[%d/%d], layer: %d, height: %f\n", i, N - 1, path[i].layer,
             path[i].height);
    }
    col_index++;
    if (i < N - 1) {
      PathPointToNode(path[i + 1], x2);
      for (int j = 0; j < interpolate_num_; ++j) {
        auto inter_x =
            GPInterpolator::Interpolate(x1, x2, kQc, dt, tau * (j + 1));
        double height_hint =
            path[i].height + (path[i + 1].height - path[i].height) * (j + 1) /
                                 (interpolate_num_ + 1);
        opt_init_value_.col(col_index) = inter_x;
        opt_init_layer_(col_index) = map_->UpdateLayerSafe(
            path[i].layer, inter_x(0, 0), inter_x(3, 0), height_hint);
        col_index++;
        if (debug_) {
          printf(
              "path[%d/%d], layer: %d, inter_layer: %f, height: %f, "
              "height2:%f, hint%f, (%f, %f)\n",
              i, opt_init_layer_.size() - 1, path[i].layer,
              opt_init_layer_(col_index), path[i].height, path[i + 1].height,
              height_hint, inter_x(0, 0), inter_x(3, 0));
        }
      }
    }
  }

  if (debug_) {
    std::cout << "opt_init_layer OK" << std::endl;
  }

  graph.add(PriorFactor6(gtsam::Symbol('x', 0), x0, sigma_initial));
  factor_idx++;
  for (int i = 1; i < N; ++i) {
    gtsam::Key last_x = gtsam::Symbol('x', i - 1);
    gtsam::Key this_x = gtsam::Symbol('x', i);
    graph.add(GPPriorFactor(last_x, this_x, dt, kQc));
    factor_idx++;
    for (int j = 0; j < interpolate_num_; ++j) {
      double height_hint =
          path[i - 1].height + (path[i].height - path[i - 1].height) * (j + 1) /
                                   (interpolate_num_ + 1);
      graph.add(GPInterpolateObstacleFactor(
          last_x, this_x, map_, path[i].layer, height_hint, safe_cost_margin_,
          0.1, kQc, dt, (i - 1) * dt, tau * (j + 1)));
      obstacle_factor_idx.emplace_back(factor_idx + 1e7);
      graph.add(GPInterpolateHeadingRateFactor(
          last_x, this_x, max_heading_rate_, kQcHeading, kQc, dt, (i - 1) * dt,
          tau * (j + 1)));
      factor_idx += 2;
    }
    if (i < N - 1) {
      graph.add(GPObstacleFactor(this_x, map_, path[i].layer, path[i].height,
                                 0.1, safe_cost_margin_, true));
      obstacle_factor_idx.emplace_back(factor_idx);
      graph.add(GPHeadingRateFactor(this_x, max_heading_rate_, kQcHeading));
      factor_idx += 2;
    }
  }
  graph.add(PriorFactor6(gtsam::Symbol('x', N - 1), xN, sigma_goal));
  factor_idx++;

  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(200.0);
  param.setMaxIterations(max_iterations_);
  // param.setAbsoluteErrorTol(5e-4);
  // param.setRelativeErrorTol(0.01);
  // param.setErrorTol(1.0);
  // param.setVerbosity("ERROR");

  gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);
  opt_results_ = Eigen::MatrixXd(N, 6);
  auto solution = opt.optimize();

  opt_layers_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);
  opt_layers_(0) = path.front().layer;
  opt_layers_(opt_layers_.size() - 1) = path.back().layer;
  for (int i = 0; i < obstacle_factor_idx.size(); ++i) {
    // printf("%d, %d, %d, %d\n", i + 1, N, obstacle_factor_idx.size(),
    //        obstacle_factor_idx[i]);
    if (obstacle_factor_idx[i] < 1e7) {
      opt_layers_(i + 1) = dynamic_cast<GPObstacleFactor*>(
                               graph.at(obstacle_factor_idx[i]).get())
                               ->GetNodeLayer();
    } else {
      opt_layers_(i + 1) = dynamic_cast<GPInterpolateObstacleFactor*>(
                               graph.at(obstacle_factor_idx[i] - 1e7).get())
                               ->GetNodeLayer();
    }
  }

  for (int i = 0; i < N; ++i) {
    opt_results_.row(i) =
        solution.at<Vector6>(gtsam::Symbol('x', i)).transpose();
  }

  WnojTrajectoryInterpolator traj_interpolator =
      WnojTrajectoryInterpolator(opt_results_, dt, kQc);
  trajectory_ = traj_interpolator.GenerateTrajectory(interpolate_num_);
  printf("shape: %d, %d\n", trajectory_.rows(), trajectory_.cols());

  printf("smooth height\n");
  opt_height_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);
  opt_ceiling_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);

  for (int i = 0; i < opt_layers_.size(); ++i) {
    opt_height_(i) =
        map_->GetHeight(opt_layers_(i), trajectory_(i, 0), trajectory_(i, 3)) +
        reference_height_;
    opt_ceiling_(i) =
        map_->GetCeiling(opt_layers_(i), trajectory_(i, 0), trajectory_(i, 3));
  }
  opt_height_ = height_smoother_.Smooth(opt_height_, opt_ceiling_, tau, N, dt);

  auto t1 = std::chrono::high_resolution_clock::now();
  printf(
      "Optimization finished for N = %d at iteration %d, time elapsed: %f ms, "
      "cost: %f\n",
      N, opt.iterations(),
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() /
          1000.0,
      opt.error());

  return true;
}

void GPMPOptimizer::PathPointToNode(const PathPoint& path_point, Vector6& x) {
  double v = std::max(path_point.ref_v, 1.0);
  x(0, 0) = path_point.x;
  x(1, 0) = std::cos(path_point.heading) * v;
  x(2, 0) = 0.0;
  x(3, 0) = path_point.y;
  x(4, 0) = std::sin(path_point.heading) * v;
  x(5, 0) = 0.0;
}

void GPMPOptimizer::SubSamplePath(const std::vector<PathPoint>& path,
                                  std::vector<PathPoint>& sub_sampled_path) {
  assert(path.size() > 1);

  if (!sub_sampled_path.empty()) {
    sub_sampled_path.clear();
  }

  int size = path.size();
  int num_segs = std::ceil((size - 1) / sample_interval_);

  if (num_segs <= 1) {
    sub_sampled_path.emplace_back(path.front());
    sub_sampled_path.emplace_back(path.back());
    return;
  }

  double new_interval = static_cast<double>(size - 1) / num_segs;

  int idx;
  double float_idx = 0.0;

  sub_sampled_path.emplace_back(path.front());

  for (int i = 1; i < num_segs + 1; ++i) {
    float_idx += new_interval;
    idx = static_cast<int>(float_idx);

    if (idx == size - 1) {
      sub_sampled_path.emplace_back(path[idx]);
    } else {
      sub_sampled_path.emplace_back(
          PathPoint::Interpolate(path[idx], path[idx + 1], float_idx - idx));
    }
  }

  printf("num segs: %d, new interval: %f\n", num_segs, new_interval);
}

Eigen::VectorXd GPMPOptimizer::GetHeadingRate() const {
  assert(trajectory_.rows() > 0);

  Eigen::VectorXd heading_rate = Eigen::VectorXd::Zero(trajectory_.rows());
  for (int i = 0; i < trajectory_.rows(); ++i) {
    double dx = trajectory_(i, 1);
    double ddx = trajectory_(i, 2);
    double dy = trajectory_(i, 4);
    double ddy = trajectory_(i, 5);
    heading_rate(i) = (ddy * dx - dy * ddx) / (dx * dx + dy * dy + 1e-6);
  }
  return heading_rate;
}