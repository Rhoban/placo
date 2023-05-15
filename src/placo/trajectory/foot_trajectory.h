#pragma once

#include <Eigen/Dense>
#include "placo/model/humanoid_parameters.h"

namespace placo
{
struct FootTrajectory
{
public:
  virtual Eigen::Vector3d pos(double t) = 0;
  virtual Eigen::Vector3d vel(double t) = 0;

  double t_start, t_end;
};
}  // namespace placo