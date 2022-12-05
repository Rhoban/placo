#pragma once

#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/model/humanoid_robot.h"
#include "placo/planning/jerk_planner.h"

namespace placo
{
class WalkPatternGenerator
{
public:
  struct Trajectory
  {
    // Planned footsteps
    std::vector<FootstepsPlanner::Footstep> footsteps;

    // CoM trajectory
    JerkPlanner::JerkTrajectory2D com;
  };

  WalkPatternGenerator(HumanoidRobot& robot);

  void planFootsteps(Trajectory &trajectory, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);
  void planCoM(Trajectory &trajectory);

  void plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  HumanoidRobot& robot;
};
}  // namespace placo