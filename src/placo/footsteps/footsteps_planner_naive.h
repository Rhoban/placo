#pragma once

#include "footsteps_planner.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo
{
class FootstepsPlannerNaive : public FootstepsPlanner
{
public:
  FootstepsPlannerNaive(HumanoidRobot::Side initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  FootstepsPlannerNaive(std::string initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  /**
   * @brief Generate the footsteps
   */
  void plan();

  /**
   * @brief Configure the naive footsteps planner
   * @param T_world_targetLeft Targetted frame for the left foot
   * @param T_world_targetRight Targetted frame for the right foot
   * @param max_steps Maximum number of steps
   * @param accessibility_width Width of accessibility window for the opposite foot
   * @param accessibility_length Length of accessibility window for the opposite foot
   * @param accessibility_yaw Yaw of accessibility window for the opposite foot
   * @param place_treshold Distance where the robot walks forward instead of aligning with target
   */
  void configure(Eigen::Affine3d T_world_targetLeft, Eigen::Affine3d T_world_targetRight, int max_steps = 100,
                 double accessibility_width = 0.025, double accessibility_length = 0.08, double accessibility_yaw = 0.2,
                 double place_threshold = 0.5);

protected:
  // Maximum steps to plan
  int max_steps;

  // Targetted position for the robot
  Eigen::Affine3d T_world_targetLeft;
  Eigen::Affine3d T_world_targetRight;

  // Dimension of the accessibility window for the opposite foot
  double accessibility_width;
  double accessibility_length;
  double accessibility_yaw;

  // Distance where the robot walks forward instead of aligning with target
  double place_threshold;
};
}  // namespace placo