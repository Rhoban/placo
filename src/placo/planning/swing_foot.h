#pragma once

#include <Eigen/Dense>
#include "placo/model/humanoid_parameters.h"

namespace placo
{
class SwingFoot
{
public:
  SwingFoot(double t_start, double t_end, double height, Eigen::Vector3d start, Eigen::Vector3d target);

  void compute_abcd(Eigen::Vector3d p0, Eigen::Vector3d m0, Eigen::Vector3d p1, Eigen::Vector3d m1);

  Eigen::Vector3d pos(double t);
  Eigen::Vector3d vel(double t);

  double t_start;
  double t_end;

protected:
  // Computed polynom
  Eigen::Vector3d a, b, c, d;
};
}  // namespace placo