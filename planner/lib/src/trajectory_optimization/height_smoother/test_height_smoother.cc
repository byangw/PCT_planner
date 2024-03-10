#include <Eigen/Dense>
#include <vector>

#include "trajectory_optimization/height_smoother/height_smoother.h"

int main(int argc, char const *argv[]) {
  HeightSmoother height_smoother;
  Eigen::VectorXd coarse_height(9);
  coarse_height << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  double dt = 1;
  double N = 3;
  double interval = 3;
  height_smoother.Smooth(coarse_height, dt, N, interval);

  return 0;
}
