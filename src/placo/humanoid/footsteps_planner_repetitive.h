#pragma once

#include "placo/humanoid/footsteps_planner.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo::humanoid
{
class FootstepsPlannerRepetitive : public FootstepsPlanner
{
public:
  FootstepsPlannerRepetitive(HumanoidParameters& parameters);

  /**
   * @brief Compute the next footsteps based on coordinates expressed in the support frame
   * laterally translated of +/- feet_spacing
   * @param x Longitudinal distance
   * @param y Lateral distance
   * @param theta Angle
   * @param steps Number of steps
   */
  void configure(double x, double y, double theta, int steps);

  /**
   * @brief Return the type of footsteps planner
   */
  std::string name();

protected:
  // Step configuration
  double d_x;
  double d_y;
  double d_theta;

  // Number of steps to plan
  int nb_steps;

  /**
   * @brief Generate the footsteps
   * @param flying_side first step side
   * @param T_world_left frame of the initial left foot
   * @param T_world_right frame of the initial right foot
   */
  void plan_impl(std::vector<Footstep>& footsteps, HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_left,
                 Eigen::Affine3d T_world_right);
};
}  // namespace placo::humanoid