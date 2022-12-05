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
  /**
   * @brief DT for planning [s]
   */
  double dt = 0.1;

  /**
   * @brief sqrt(g/h): constant for pendulum-based points (ZMP and DCM)
   */
  double omega = 0.0;

  /**
   * @brief SSP duration [ms], must be a multiple of dt
   */
  double single_support_duration = 0.3;

  /**
   * @brief DSP duration [ms], must be a multiple of dt
   */
  double double_support_duration = 0.3;

  /**
   * @brief Maximum steps for planning
   */
  int maximum_steps = 100;

  struct Trajectory
  {
    // Planned footsteps
    std::vector<FootstepsPlanner::Support> footsteps;

    // CoM trajectory
    JerkPlanner::JerkTrajectory2D com;
  };

  WalkPatternGenerator(HumanoidRobot& robot);

  void planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);
  void planCoM(Trajectory& trajectory);

  Trajectory plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  HumanoidRobot& robot;
};
}  // namespace placo