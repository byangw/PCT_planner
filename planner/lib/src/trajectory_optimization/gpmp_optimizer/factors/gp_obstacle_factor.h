#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"

class GPObstacleFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 public:
  GPObstacleFactor(gtsam::Key key, std::shared_ptr<DenseElevationMap> map,
                   int current_layer, const double height_hint,
                   const double q_cost, const double cost_threshold,
                   bool verbose = false)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        current_layer_(current_layer),
        height_hint_(height_hint),
        cost_threshold_(cost_threshold),
        map_(map),
        verbose_(verbose) {}
  ~GPObstacleFactor() = default;

  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

  void verbose() { verbose_ = true; }

 private:
  bool verbose_ = false;
  mutable double height_hint_ = 0.0;
  mutable int current_layer_ = 0;
  double cost_threshold_ = 0.0;
  std::shared_ptr<DenseElevationMap> map_;
};