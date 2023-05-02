#pragma once

#include <Eigen/Dense>
#include "placo/model/humanoid_parameters.h"
#include "placo/problem/problem.h"
#include "placo/problem/integrator.h"

namespace placo
{
class SwingFootQP
{
public:
  struct Trajectory
  {
    Eigen::Vector3d pos(double t);
    Eigen::Vector3d vel(double t);

    Integrator::Trajectory x;
    Integrator::Trajectory y;
    Integrator::Trajectory z;
  };

  static Trajectory make_trajectory(double t_start, double t_end, double min_height, double max_height,
                                    Eigen::Vector3d start, Eigen::Vector3d target, int timesteps = 12);
};
}  // namespace placo