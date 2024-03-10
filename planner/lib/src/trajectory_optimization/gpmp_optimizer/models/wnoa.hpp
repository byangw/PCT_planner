#pragma once

#include <Eigen/Core>

class GPConstVelocityModel1D {
 public:
  static inline Eigen::Matrix2d Q(const double Qc, const double tau) {
    static Eigen::Matrix2d q;
    constexpr double one_third = 1.0 / 3.0;
    q(0, 0) = one_third * std::pow(tau, 3) * Qc;
    q(0, 1) = 0.5 * std::pow(tau, 2) * Qc;
    q(1, 0) = q(0, 1);
    q(1, 1) = tau * Qc;
    return q;
  }

  static inline Eigen::Matrix2d Phi(double tau) {
    static Eigen::Matrix2d phi = Eigen::Matrix2d::Identity();
    phi(0, 1) = tau;
    return phi;
  }

  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix2d* lambda,
                                  Eigen::Matrix2d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

 private:
  static inline Eigen::Matrix2d QInverse(const double Qc, const double tau) {
    static Eigen::Matrix2d q_inv;
    const double qc_inv = 1.0 / Qc;
    q_inv(0, 0) = 12 * std::pow(tau, -3) * qc_inv;
    q_inv(0, 1) = -6 * std::pow(tau, -2) * qc_inv;
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 4 / tau * qc_inv;
    return q_inv;
  }
};

class WhiteNoiseOnAcceleration2D {
 public:
  static inline Eigen::Matrix4d Q(const double Qc, const double tau) {
    static Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
    constexpr double one_third = 1.0 / 3.0;

    q(0, 0) = one_third * std::pow(tau, 3) * Qc;
    q(0, 1) = 0.5 * std::pow(tau, 2) * Qc;
    q(1, 0) = q(0, 1);
    q(1, 1) = tau * Qc;

    q(2, 2) = q(0, 0);
    q(2, 3) = q(0, 1);
    q(3, 2) = q(1, 0);
    q(3, 3) = q(1, 1);

    return q;
  }

  static inline Eigen::Matrix4d Phi(double tau) {
    static Eigen::Matrix4d phi = Eigen::Matrix4d::Identity();
    phi(0, 1) = tau;
    phi(2, 3) = tau;
    return phi;
  }

  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix4d* lambda,
                                  Eigen::Matrix4d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

 private:
  static inline Eigen::Matrix4d QInverse(const double Qc, const double tau) {
    static Eigen::Matrix4d q_inv = Eigen::Matrix4d::Zero();
    const double qc_inv = 1.0 / Qc;
    q_inv(0, 0) = 12 * std::pow(tau, -3) * qc_inv;
    q_inv(0, 1) = -6 * std::pow(tau, -2) * qc_inv;
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 4 / tau * qc_inv;

    q_inv(2, 2) = q_inv(0, 0);
    q_inv(2, 3) = q_inv(0, 1);
    q_inv(3, 2) = q_inv(1, 0);
    q_inv(3, 3) = q_inv(1, 1);
    return q_inv;
  }
};