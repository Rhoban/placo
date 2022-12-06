#pragma once

namespace placo
{
/**
 * @brief A collection of parameters that can be used to define the capabilities and the constants behind
 * planning and control of an humanoid robot.
 *
 * Constants from this dataclass are used by the solvers to parametrize them.
 */
class HumanoidParameters
{
public:
  /**
   * @brief dt for planning [s]
   */
  double dt = 0.1;

  /**
   * @brief SSP duration [ms], must be a multiple of dt
   */
  double single_support_duration = 1.;

  /**
   * @brief DSP duration [ms], must be a multiple of dt
   */
  double double_support_duration = 1.;

  /**
   * @brief Maximum steps for planning
   */
  int maximum_steps = 100;

  /**
   * @brief Margin for the ZMP to live in the support polygon [m]
   */
  double zmp_margin = 0.05;

  /**
   * @brief How height the feet are rising while walking
   */
  double walk_foot_height = 0.03;

  /**
   * @brief Robot center of mass height for LIPM model. This is used to compute the pendulum constant
   * omega, which is sqrt(g/h)
   */
  double pendulum_height = 0.0;

  /**
   * @brief Lateral spacing between feet [m]
   */
  double feet_spacing = 0.1;

  /**
   * @brief Foot width [m]
   */
  double foot_width = 0.1;

  /**
   * @brief Foot length [m]
   */
  double foot_length = 0.15;

  double omega();
};
}  // namespace placo