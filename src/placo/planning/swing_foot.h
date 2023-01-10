#pragma once

#include <Eigen/Dense>
#include "placo/model/humanoid_parameters.h"

namespace placo
{
class SwingFoot
{
public:
  struct Trajectory
  {
    Eigen::Vector3d pos(double t);
    Eigen::Vector3d vel(double t);

    void compute_abcd(Eigen::Vector3d p0, Eigen::Vector3d m0, Eigen::Vector3d p1, Eigen::Vector3d m1);

    // Computed polynom (ax^3 + bx^2 + cx + d)
    Eigen::Vector3d a, b, c, d;

    double t_start, t_end;
  };

  static Trajectory make_trajectory(double t_start, double t_end, double height, Eigen::Vector3d start,
                                    Eigen::Vector3d target);
};
}  // namespace placo