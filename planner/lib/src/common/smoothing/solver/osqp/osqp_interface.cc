#include "common/smoothing/solver/osqp/osqp_interface.h"

#include <iostream>

#include "common/smoothing/solver/osqp/osqp_sparse_matrix.h"

namespace common {

bool OsqpInterface::Solve(
    const ColSparseMatrix& P,
    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> q,
    const ColSparseMatrix& A,
    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l,
    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u,
    Eigen::VectorXd* x) {
  OSQPSettings* settings =
      static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  data->P = nullptr;
  data->A = nullptr;
  data->n = P.rows();
  data->m = A.rows();
  ColSparseMatrix P_uppper_triangular = P.triangularView<Eigen::Upper>();
  if (!CreateOsqpSparseMatrix(P_uppper_triangular, data->P)) {
    return false;
  }

  if (!CreateOsqpSparseMatrix(A, data->A)) {
    return false;
  }

  data->q = q.data();
  data->l = l.data();
  data->u = u.data();

  osqp_set_default_settings(settings);
  settings->alpha = 1.0;
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polish = true;
  settings->verbose = false;

  OSQPWorkspace* work;
  c_int exitflag = osqp_setup(&work, data, settings);

  osqp_solve(work);
  auto status = work->info->status_val;

  if (status < 0 || (status != 1 && status != 2)) {
    // LOG(ERROR) << "failed optimization status:\t" << work->info->status;
    return false;
  } else if (work->solution == nullptr) {
    // LOG(ERROR) << "The solution from OSQP is nullptr";
    return false;
  }

  c_float* solution = work->solution->x;
  (*x) = Eigen::Map<Eigen::VectorXd>(solution, work->data->n, 1);

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}

bool OsqpInterface::Solve(
    const Eigen::MatrixXd& P,
    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> q,
    const Eigen::MatrixXd& A,
    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l,
    Eigen::Ref<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u,
    Eigen::VectorXd* x) {
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  const Eigen::MatrixXd& P_upper = P.triangularView<Eigen::Upper>();
  DenseToCSCMatrix(P_upper, &P_data, &P_indices, &P_indptr);

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  OSQPSettings* settings =
      static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  data->n = P.rows();
  data->m = A.rows();
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = l.data();
  data->u = u.data();

  osqp_set_default_settings(settings);
  settings->alpha = 1.0;
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polish = true;
  settings->verbose = false;

  OSQPWorkspace* work;
  c_int exitflag = osqp_setup(&work, data, settings);

  osqp_solve(work);
  auto status = work->info->status_val;

  if (status < 0 || (status != 1 && status != 2)) {
    // LOG(ERROR) << "failed optimization status:\t" << work->info->status;
    return false;
  } else if (work->solution == nullptr) {
    // LOG(ERROR) << "The solution from OSQP is nullptr";
    return false;
  }

  c_float* solution = work->solution->x;
  (*x) = Eigen::Map<Eigen::VectorXd>(solution, work->data->n, 1);

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}
}  // namespace common