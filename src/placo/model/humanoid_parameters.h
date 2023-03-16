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
   * @brief DSP duration [ms], must be a multiple of dt
   */
  double startend_double_support_duration = 1.;

  /**
   * @brief Kick duration [ms], must be a multiple of dt
   */
  double kick_duration = 1.;

  /**
   * @brief Maximum planned dt
   */
  int planned_dt = 100;

  /**
   * @brief Number of dt between each replan
   */
  int replan_frequency = 10;

  /**
   * @brief Margin for the ZMP to live in the support polygon [m]
   */
  double zmp_margin = 0.025;

  /**
   * @brief How height the feet are rising while walking [m]
   */
  double walk_foot_height = 0.05;

  /**
   * @brief CoM height while walking [m]
   */
  double walk_com_height = 0.4;

  /**
   * @brief Trunk pitch while walking [rad]
   */
  double walk_trunk_pitch = 0.0;

  /**
   * @brief How muc hthe foot tilts during the walk [rad]
   */
  double walk_foot_tilt = 0.2;

  /**
   * @brief Robot center of mass height for LIPM model. This is used to compute the pendulum constant
   * omega, which is sqrt(g/h)
   *
   * A higher pendulum height results in less left/right body swinging during the walk.
   */
  double pendulum_height = 0.4;

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