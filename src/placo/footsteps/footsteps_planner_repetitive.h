#pragma once

#include "footsteps_planner.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo
{
class FootstepsPlannerRepetitive : public FootstepsPlanner
{
public:
  FootstepsPlannerRepetitive(HumanoidRobot::Side initial_side, Eigen::Affine3d T_world_left,
                             Eigen::Affine3d T_world_right);

  FootstepsPlannerRepetitive(std::string initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  /**
   * @brief Generate the footsteps
   */
  void plan();

  /// @brief Compute the next footsteps based on coordinates expressed in the support frame
  /// laterally translated of +/- feet_spacing
  /// @param d_x Longitudinal distance
  /// @param d_y Lateral distance
  /// @param d_theta Angle
  /// @param nb_steps Number of steps
  void configure(double d_x, double d_y, double d_theta, int nb_steps);

protected:
  // Step configuration
  double d_x;
  double d_y;
  double d_theta;

  // Number of steps to plan
  int nb_steps;

  // Maximum absolute value of d_x in meters
  double max_d_x = 0.2;

  // Maximum absolute value of d_y in meters
  double max_d_y = 0.1;

  // Maximum absolute value of d_theta in radians
  double max_d_theta = 0.785;
};
}  // namespace placo