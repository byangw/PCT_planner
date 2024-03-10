#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_interpolator.hpp"

class GPInterpolateObstacleFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6> {
 public:
  GPInterpolateObstacleFactor(gtsam::Key key1, gtsam::Key key2,
                              std::shared_ptr<DenseElevationMap> map,
                              const int current_layer, const double height_hint,
                              const double cost_threshold, const double q_cost,
                              const double qc, const double interval,
                              const double param_start, const double tau)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key1,
                          key2),
        current_layer_(current_layer),
        height_hint_(height_hint),
        map_(map),
        cost_threshold_(cost_threshold),
        param_(param_start + tau),
        tau_(tau),
        gp_interpolator_(qc, interval, tau) {}
  ~GPInterpolateObstacleFactor() = default;

  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  mutable int count_ = 0;
  mutable bool initialized_ = false;
  mutable int current_layer_ = 0;
  mutable double height_hint_ = 0.0;
  double cost_threshold_ = 0.0;
  double tau_ = 0.0;
  double param_ = 0.0;

  GPInterpolator gp_interpolator_;
  std::shared_ptr<DenseElevationMap> map_;
};