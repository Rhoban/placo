#pragma once

#include "placo/trajectory/cubic_spline.h"
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"
#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/planning/lipm.h"

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

  CubicSpline3D left_foot_trajectory;
  CubicSpline3D right_foot_trajectory;

  LIPM::Trajectory com_trajectory;
  CubicSpline com_height;

  double duration;

  bool support_is_both(double t);
  HumanoidRobot::Side support_side;
  Eigen::Affine3d support_frame;

  // Kick parameters
  double kick_com_height = 0.3;
  double kick_foot_height = 0.1;
  double t_init = 1;
  double t_pre_delay = 0.5;
  double t_up = 0.3;
  double t_post_delay = 0.5;

protected:
  // Robot associated to the WPG
  HumanoidRobot& robot;

  // The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
  HumanoidParameters& parameters;

  Eigen::Matrix3d R_world_left;
  Eigen::Matrix3d R_world_right;
};
}  // namespace placo