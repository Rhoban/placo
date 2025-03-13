#pragma once

#include <Eigen/Dense>
#include "placo/humanoid/humanoid_parameters.h"
#include "placo/tools/cubic_spline.h"
#include "placo/humanoid/foot_trajectory.h"

namespace placo::humanoid
{
/**
 * @brief Cubic swing foot
 */
class SwingFootCubic
{
public:
  struct Trajectory : FootTrajectory
  {
    virtual Eigen::Vector3d pos(double t);
    virtual Eigen::Vector3d vel(double t);

    placo::tools::CubicSpline x;
    placo::tools::CubicSpline y;
    placo::tools::CubicSpline z;

    double t_start;
    double virt_duration;
  };

  static Trajectory make_trajectory(double t_start, double virt_duration, double height, double rise_ratio, Eigen::Vector3d start, 
    Eigen::Vector3d target, double elapsed_ratio = 0., Eigen::Vector3d start_vel = Eigen::Vector3d::Zero());
};
}  // namespace placo::humanoid