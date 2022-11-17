#pragma once

#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/model/humanoid_robot.h"

namespace placo
{
class WalkPatternGenerator
{
public:
  WalkPatternGenerator(HumanoidRobot& robot);

  void plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  HumanoidRobot& robot;

  // Planned footsteps
  std::vector<FootstepsPlanner::Footstep> footsteps;
};
}  // namespace placo