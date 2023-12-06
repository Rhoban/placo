#pragma once

#include <Eigen/Dense>
#include "placo/humanoid/humanoid_parameters.h"

namespace placo::humanoid
{
struct FootTrajectory
{
public:
  virtual ~FootTrajectory();
  virtual Eigen::Vector3d pos(double t) = 0;
  virtual Eigen::Vector3d vel(double t) = 0;

  double t_start, t_end;
};
}  // namespace placo::humanoid