#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_interpolator.hpp"

class GPInterpolateHeadingRateFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6> {
 public:
  GPInterpolateHeadingRateFactor(gtsam::Key key1, gtsam::Key key2,
                                 const double max_heading_rate,
                                 const double q_cost, const double qc,
                                 const double interval,
                                 const double param_start, const double tau)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key1,
                          key2),
        max_heading_rate_(max_heading_rate),
        param_(param_start + tau),
        tau_(tau),
        gp_interpolator_(qc, interval, tau) {}
  ~GPInterpolateHeadingRateFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  double max_heading_rate_ = 0.5;
  double tau_ = 0.0;
  double param_ = 0.0;

  GPInterpolator gp_interpolator_;
};