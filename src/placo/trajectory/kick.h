#pragma once

#include "placo/trajectory/foot_trajectory.h"
#include "placo/trajectory/cubic_spline_3d.h"
#include "placo/model/humanoid_robot.h"

// #include "placo/model/humanoid_parameters.h"
// #include "placo/planning/walk_pattern_generator.h"
// #include "placo/footsteps/footsteps_planner.h"
// #include "placo/planning/lipm.h"

namespace placo::trajectory
{
class Kick
{
public:
  struct KickTrajectory : FootTrajectory
  {
    Eigen::Vector3d pos(double t);
    Eigen::Vector3d vel(double t);

    CubicSpline3D foot_trajectory;
  };

  static KickTrajectory make_trajectory(HumanoidRobot::Side kicking_side, double t_start, double t_end,
                                        Eigen::Vector3d start, Eigen::Vector3d target, Eigen::Affine3d T_world_opposite,
                                        HumanoidParameters& parameters);

  // double kicking_com_height = 0.32;
  // double feet_spacing = 0.12;
  // double com_support_offset = 0.02;
};
}  // namespace placo::trajectory