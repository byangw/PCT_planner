#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/Symbol.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_prior_factor.h"

using gtsam::Vector6;
using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using PriorFactor6 = gtsam::PriorFactor<Vector6>;

constexpr double kQc = 0.1;

int main(int argc, char const *argv[]) {
  static auto sigma_initial = Isotropic::Sigma(6, 0.001);
  static auto sigma_goal = Diagonal::Sigmas(Eigen::VectorXd::Ones(6) * 0.001);

  double T = 10;
  int N = 21;
  double dt = T / (N - 1);

  auto graph = gtsam::NonlinearFactorGraph();

  gtsam::Vector6 x0 = gtsam::Vector6::Zero();
  gtsam::Vector6 xN = gtsam::Vector6::Zero();
  xN(0) = 10;
  xN(3) = 10;

  graph.add(PriorFactor6(gtsam::Symbol('x', 0), x0, sigma_initial));
  for (int i = 1; i < N - 1; ++i) {
    gtsam::Key last_x = gtsam::Symbol('x', i - 1);
    gtsam::Key this_x = gtsam::Symbol('x', i);
    graph.add(GPPriorFactor(last_x, this_x, dt, 0.1));
  }
  graph.add(PriorFactor6(gtsam::Symbol('x', N - 1), xN, sigma_goal));

  gtsam::Values init_values;
  for (int i = 0; i < N; ++i) {
    init_values.insert<Vector6>(gtsam::Symbol('x', i),
                                x0 + i * (xN - x0) / (N - 1));
  }

  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(100.0);
  param.setMaxIterations(50);
  // param.setAbsoluteErrorTol(5e-4);
  // param.setRelativeErrorTol(0.01);
  // param.setErrorTol(1.0);
  // param.setVerbosity("ERROR");

  gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);
  auto solution = opt.optimize();
  for (int i = 0; i < N; ++i) {
    std::cout << solution.at<Vector6>(gtsam::Symbol('x', i)).transpose()
              << std::endl;
  }

  return 0;
}
