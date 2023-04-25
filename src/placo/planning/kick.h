#pragma once

#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"
#include "placo/planning/walk_pattern_generator.h"
#include "placo/planning/jerk_planner.h"
#include "placo/footsteps/footsteps_planner.h"

namespace placo
{
class Kick
{
public:
  Kick(HumanoidRobot& robot, HumanoidParameters& parameters);

  void one_foot_balance(FootstepsPlanner& planner, HumanoidRobot::Side support_side);
  // void kick(HumanoidRobot::Side kicking_side);

  Eigen::Affine3d get_T_world_left(double t);
  Eigen::Affine3d get_T_world_right(double t);
  Eigen::Vector3d get_com_world(double t);

  rhoban_utils::PolySpline3D left_foot_trajectory;
  rhoban_utils::PolySpline3D right_foot_trajectory;

  JerkPlanner::JerkTrajectory2D com_trajectory;
  rhoban_utils::PolySpline com_height;

  double duration;

  // Kick parameters
  double kick_com_height = 0.3;
  double t_init = 1;
  double t_up = 0.3;

protected:
  // Robot associated to the WPG
  HumanoidRobot& robot;

  // The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
  HumanoidParameters& parameters;
};
}  // namespace placo