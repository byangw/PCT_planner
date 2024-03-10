#pragma once

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoj.hpp"

class GPPriorFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector6, gtsam::Vector6> {
 public:
  GPPriorFactor(gtsam::Key key1, gtsam::Key key2, const double delta,
                const double Qc)
      : NoiseModelFactor2(gtsam::noiseModel::Gaussian::Covariance(
                              WhiteNoiseOnJerkModel2D::Q(Qc, delta)),
                          key1, key2),
        delta_(delta),
        phi_(WhiteNoiseOnJerkModel2D::Phi(delta)){};
  ~GPPriorFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

  void verbose() {}

 private:
  double delta_ = 0.0;
  gtsam::Matrix66 phi_;
};