#pragma once

#include <osqp/osqp.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace common {

using ColSparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor>;

/**
 * @class OsqpInterface (ADMM based qp solver)
 * @brief solve convex quadratic programming problem
 *
 * min_{x} 1/2 x'Px + q'x
 * s.t.    l <= Ax <= u
 *
 * Note: osqp use sparse matrix representation
 * [compressed-column](https://people.sc.fsu.edu/~jburkardt/data/cc/cc.html)
 */
class OsqpInterface {
 public:
  // please notice that q, l, u may be modified during optimization
  static bool Solve(const ColSparseMatrix& P,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> q,
                    const ColSparseMatrix& A,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u,
                    Eigen::VectorXd* x);

  // Although dense matrix interface is provided, one should notice that dense
  // matrix P, A will be convert to sparse matrix anyway.
  static bool Solve(const Eigen::MatrixXd& P,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> q,
                    const Eigen::MatrixXd& A,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l,
                    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u,
                    Eigen::VectorXd* x);
};

}  // namespace common