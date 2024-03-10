#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"

class GPHeadingRateFactor : public gtsam::NoiseModelFactor1<gtsam::Vector6> {
 public:
  GPHeadingRateFactor(gtsam::Key key, const double max_heading_rate,
                      const double q_cost)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        max_heading_rate_(max_heading_rate) {}
  ~GPHeadingRateFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

 private:
  double max_heading_rate_ = 0.5;
};