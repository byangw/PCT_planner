#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

#include <cmath>

/// get Qc covariance matrix from noise model
gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model);

/// calculate Q
inline gtsam::Matrix calcQ(const gtsam::Matrix& Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  return (gtsam::Matrix(2 * Qc.rows(), 2 * Qc.rows())
              << 1.0 / 3 * pow(tau, 3.0) * Qc,
          1.0 / 2 * pow(tau, 2.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc)
      .finished();
}

/// calculate Q_inv
inline gtsam::Matrix calcQ_inv(const gtsam::Matrix& Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const gtsam::Matrix Qc_inv = Qc.inverse();
  return (gtsam::Matrix(2 * Qc.rows(), 2 * Qc.rows())
              << 12.0 * pow(tau, -3.0) * Qc_inv,
          (-6.0) * pow(tau, -2.0) * Qc_inv, (-6.0) * pow(tau, -2.0) * Qc_inv,
          4.0 * pow(tau, -1.0) * Qc_inv)
      .finished();
}

/// calculate Phi
inline gtsam::Matrix calcPhi(size_t dof, double tau) {
  return (gtsam::Matrix(2 * dof, 2 * dof) << gtsam::Matrix::Identity(dof, dof),
          tau * gtsam::Matrix::Identity(dof, dof),
          gtsam::Matrix::Zero(dof, dof), gtsam::Matrix::Identity(dof, dof))
      .finished();
}

/// calculate Lambda
inline gtsam::Matrix calcLambda(const gtsam::Matrix& Qc, double delta_t,
                                const double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcPhi(Qc.rows(), tau) -
         calcQ(Qc, tau) * (calcPhi(Qc.rows(), delta_t - tau).transpose()) *
             calcQ_inv(Qc, delta_t) * calcPhi(Qc.rows(), delta_t);
}

/// calculate Psi
inline gtsam::Matrix calcPsi(const gtsam::Matrix& Qc, double delta_t,
                             double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcQ(Qc, tau) * (calcPhi(Qc.rows(), delta_t - tau).transpose()) *
         calcQ_inv(Qc, delta_t);
}
