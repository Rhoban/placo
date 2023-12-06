#pragma once

#include "placo/humanoid/footsteps_planner.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo::humanoid
{
class FootstepsPlannerNaive : public FootstepsPlanner
{
public:
  FootstepsPlannerNaive(HumanoidParameters& parameters);

  /**
   * @brief Return the type of footsteps planner
   */
  std::string name();

  /**
   * @brief Configure the naive footsteps planner
   * @param T_world_left_target Targetted frame for the left foot
   * @param T_world_right_target Targetted frame for the right foot
   */
  void configure(Eigen::Affine3d T_world_left_target, Eigen::Affine3d T_world_right_target);

protected:
  // Maximum steps to plan
  int max_steps = 100;

  // Targetted position for the robot
  Eigen::Affine3d T_world_targetLeft;
  Eigen::Affine3d T_world_targetRight;

  // Dimension of the accessibility window for the opposite foot
  double accessibility_width = 0.025;
  double accessibility_length = 0.08;
  double accessibility_yaw = 0.2;

  // Distance where the robot walks forward instead of aligning with target
  double place_threshold = 0.5;

  /**
   * @brief Generate the footsteps
   * @param flying_side first step side
   * @param T_world_left frame of the initial left foot
   * @param T_world_right frame of the initial right foot
   */
  void plan_impl(std::vector<Footstep>& foosteps, HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_left,
                 Eigen::Affine3d T_world_right);
};
}  // namespace placo::humanoid