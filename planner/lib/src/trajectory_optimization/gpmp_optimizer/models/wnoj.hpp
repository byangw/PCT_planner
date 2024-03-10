#pragma once

#include <Eigen/Core>
#include <array>

using Matrix6d = Eigen::Matrix<double, 6, 6>;

class WhiteNoiseOnJerkModel1D {
 public:
  static inline Eigen::Matrix3d Q(const double qc, const double tau) {
    Eigen::Matrix3d q = Eigen::Matrix3d::Zero();
    std::array<double, 5> tau_powers;
    constexpr double one_third = 1.0 / 3.0;
    constexpr double one_sixth = 1.0 / 6.0;

    tau_powers[0] = tau * qc;
    for (int i = 1; i < 5; ++i) tau_powers[i] = tau * tau_powers[i - 1];

    q(0, 0) = 0.05 * tau_powers[4];
    q(0, 1) = 0.125 * tau_powers[3];
    q(0, 2) = one_sixth * tau_powers[2];
    q(1, 0) = q(0, 1);
    q(1, 1) = one_third * tau_powers[2];
    q(1, 2) = 0.5 * tau_powers[1];
    q(2, 0) = q(0, 2);
    q(2, 1) = q(1, 2);
    q(2, 2) = tau_powers[0];
    return q;
  }

  static inline Eigen::Matrix3d Phi(const double tau) {
    Eigen::Matrix3d phi = Eigen::Matrix3d::Identity();
    phi(0, 1) = tau;
    phi(0, 2) = 0.5 * tau * tau;
    phi(1, 2) = tau;
    return phi;
  }

  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix3d* lambda,
                                  Eigen::Matrix3d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

  static inline Eigen::Matrix3d QInverse(const double qc, const double tau) {
    Eigen::Matrix3d q_inv = Eigen::Matrix3d::Zero();
    std::array<double, 5> tau_powers;
    const double tau_inv = 1.0 / tau;

    tau_powers[0] = tau_inv / qc;
    for (int i = 1; i < 5; ++i) tau_powers[i] = tau_inv * tau_powers[i - 1];

    q_inv(0, 0) = 720 * tau_powers[4];
    q_inv(0, 1) = -360 * tau_powers[3];
    q_inv(0, 2) = 60 * tau_powers[2];
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 192 * tau_powers[2];
    q_inv(1, 2) = -36 * tau_powers[1];
    q_inv(2, 0) = q_inv(0, 2);
    q_inv(2, 1) = q_inv(1, 2);
    q_inv(2, 2) = 9 * tau_powers[0];
    return q_inv;
  }
};

class WhiteNoiseOnJerkModel2D {
 public:
  static inline Matrix6d Q(const double qc, const double tau) {
    Matrix6d q = Matrix6d::Zero();
    std::array<double, 5> tau_powers;
    constexpr double one_third = 1.0 / 3.0;
    constexpr double one_sixth = 1.0 / 6.0;

    tau_powers[0] = tau * qc;
    for (int i = 1; i < 5; ++i) tau_powers[i] = tau * tau_powers[i - 1];

    q(0, 0) = 0.05 * tau_powers[4];
    q(0, 1) = 0.125 * tau_powers[3];
    q(0, 2) = one_sixth * tau_powers[2];
    q(1, 0) = q(0, 1);
    q(1, 1) = one_third * tau_powers[2];
    q(1, 2) = 0.5 * tau_powers[1];
    q(2, 0) = q(0, 2);
    q(2, 1) = q(1, 2);
    q(2, 2) = tau_powers[0];

    q(3, 3) = q(0, 0);
    q(3, 4) = q(0, 1);
    q(3, 5) = q(0, 2);
    q(4, 3) = q(1, 0);
    q(4, 4) = q(1, 1);
    q(4, 5) = q(1, 2);
    q(5, 3) = q(2, 0);
    q(5, 4) = q(2, 1);
    q(5, 5) = q(2, 2);
    return q;
  }

  static inline Matrix6d Phi(const double tau) {
    Matrix6d phi = Matrix6d::Identity();
    phi(0, 1) = tau;
    phi(0, 2) = 0.5 * tau * tau;
    phi(1, 2) = tau;

    phi(3, 4) = tau;
    phi(3, 5) = 0.5 * tau * tau;
    phi(4, 5) = tau;
    return phi;
  }

  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Matrix6d* lambda,
                                  Matrix6d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

  static inline Matrix6d QInverse(const double qc, const double tau) {
    Matrix6d q_inv = Matrix6d::Zero();
    std::array<double, 5> tau_powers;
    const double tau_inv = 1.0 / tau;

    tau_powers[0] = tau_inv / qc;
    for (int i = 1; i < 5; ++i) tau_powers[i] = tau_inv * tau_powers[i - 1];

    q_inv(0, 0) = 720 * tau_powers[4];
    q_inv(0, 1) = -360 * tau_powers[3];
    q_inv(0, 2) = 60 * tau_powers[2];
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 192 * tau_powers[2];
    q_inv(1, 2) = -36 * tau_powers[1];
    q_inv(2, 0) = q_inv(0, 2);
    q_inv(2, 1) = q_inv(1, 2);
    q_inv(2, 2) = 9 * tau_powers[0];

    q_inv(3, 3) = q_inv(0, 0);
    q_inv(3, 4) = q_inv(0, 1);
    q_inv(3, 5) = q_inv(0, 2);
    q_inv(4, 3) = q_inv(1, 0);
    q_inv(4, 4) = q_inv(1, 1);
    q_inv(4, 5) = q_inv(1, 2);
    q_inv(5, 3) = q_inv(2, 0);
    q_inv(5, 4) = q_inv(2, 1);
    q_inv(5, 5) = q_inv(2, 2);
    return q_inv;
  }
};