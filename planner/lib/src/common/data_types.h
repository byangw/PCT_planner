#pragma once

#include <Eigen/Dense>
#include <vector>

using Vector4 = Eigen::Matrix<double, 4, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

struct PathPoint {
  int layer = 0;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double ref_v = 0.0;
  double height = 0.0;

  PathPoint() = default;
  PathPoint(int layer, double x, double y, double heading, double ref_v)
      : layer(layer), x(x), y(y), heading(heading), ref_v(ref_v) {}

  static PathPoint Interpolate(const PathPoint& p1, const PathPoint& p2,
                               double t) {
    PathPoint p;
    p.layer = p1.layer;
    p.x = p1.x + (p2.x - p1.x) * t;
    p.y = p1.y + (p2.y - p1.y) * t;
    p.heading = p2.heading;
    p.height = p1.height + (p2.height - p1.height) * t;
    p.ref_v = p1.ref_v + (p2.ref_v - p1.ref_v) * t;
    return p;
  }

  void DebugString() {
    printf("layer: %d, x: %f, y: %f, heading: %f, ref_v: %f, height: %f\n",
           layer, x, y, heading, ref_v, height);
  }
};