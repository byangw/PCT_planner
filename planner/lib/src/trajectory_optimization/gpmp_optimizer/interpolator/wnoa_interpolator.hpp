#pragma once

#include "gtsam/base/Matrix.h"
#include "trajectory_optimization/gpmp_optimizer/models/wnoa.hpp"

class GPInterpolatorWnoa {
 public:
  GPInterpolatorWnoa() = default;
  GPInterpolatorWnoa(const double qc, const double interval, const double tau) {
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, interval, tau, &lambda_,
                                             &psi_);
  }
  GPInterpolatorWnoa(const double interval, const double qc)
      : interval_(interval), qc_(qc) {}
  ~GPInterpolatorWnoa() = default;

  static gtsam::Vector4 Interpolate(const gtsam::Vector4& x1,
                                    const gtsam::Vector4& x2, const double qc,
                                    const double interval, const double tau) {
    gtsam::Matrix44 lambda, psi;
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }

  inline gtsam::Vector4 Interpolate(
      const gtsam::Vector4& x1, const gtsam::Vector4& x2,
      gtsam::OptionalJacobian<4, 4> H1 = boost::none,
      gtsam::OptionalJacobian<4, 4> H2 = boost::none) const {
    if (H1) *H1 = lambda_;
    if (H2) *H2 = psi_;
    return lambda_ * x1 + psi_ * x2;
  }

  void Interpolate(const gtsam::Vector4& x1, const gtsam::Vector4& x2,
                   const double tau, gtsam::Vector4* res) const {
    gtsam::Matrix44 lambda, psi;
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc_, interval_, tau, &lambda,
                                             &psi);
    (*res) = lambda * x1 + psi * x2;
  }

  inline const gtsam::Matrix44& Lambda() const { return lambda_; }
  inline const gtsam::Matrix44& Psi() const { return psi_; }

 private:
  double interval_ = 0.0;
  double qc_ = 0.0;
  gtsam::Matrix44 lambda_;
  gtsam::Matrix44 psi_;
};

class GPInterpolatorWnoaTmp {
 public:
  GPInterpolatorWnoaTmp() = default;
  ~GPInterpolatorWnoaTmp() = default;

  gtsam::Vector4 Interpolate(const gtsam::Vector4& x1, const gtsam::Vector4& x2,
                             const double qc, const double interval,
                             const double tau) {
    gtsam::Matrix44 lambda, psi;
    WhiteNoiseOnAcceleration2D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }
};