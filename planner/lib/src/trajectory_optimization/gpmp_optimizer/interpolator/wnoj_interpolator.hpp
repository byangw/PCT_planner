#pragma once

#include "gtsam/base/Matrix.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoj.hpp"

class GPInterpolator {
 public:
  GPInterpolator() = default;
  GPInterpolator(const double qc, const double interval, const double tau) {
    WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc, interval, tau, &lambda_, &psi_);
  }
  GPInterpolator(const double interval, const double qc)
      : interval_(interval), qc_(qc) {}
  ~GPInterpolator() = default;

  static gtsam::Vector6 Interpolate(const gtsam::Vector6& x1,
                                    const gtsam::Vector6& x2, const double qc,
                                    const double interval, const double tau) {
    gtsam::Matrix66 lambda, psi;
    WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }

  inline gtsam::Vector6 Interpolate(
      const gtsam::Vector6& x1, const gtsam::Vector6& x2,
      gtsam::OptionalJacobian<6, 6> H1 = boost::none,
      gtsam::OptionalJacobian<6, 6> H2 = boost::none) const {
    if (H1) *H1 = lambda_;
    if (H2) *H2 = psi_;
    return lambda_ * x1 + psi_ * x2;
  }

  void Interpolate(const gtsam::Vector6& x1, const gtsam::Vector6& x2,
                   const double tau, gtsam::Vector6* res) const {
    gtsam::Matrix66 lambda, psi;
    WhiteNoiseOnJerkModel2D::LambdaAndPsi(qc_, interval_, tau, &lambda, &psi);
    (*res) = lambda * x1 + psi * x2;
  }

  inline const gtsam::Matrix66& Lambda() const { return lambda_; }
  inline const gtsam::Matrix66& Psi() const { return psi_; }

 private:
  double interval_ = 0.0;
  double qc_ = 0.0;
  gtsam::Matrix66 lambda_;
  gtsam::Matrix66 psi_;
};