#pragma once

#include <Eigen/Dense>

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
  double dt();

  /**
   * @brief SSP duration [ms], must be a multiple of dt
   */
  double single_support_duration = 1.;

  /**
   * @brief Number of timesteps for one single support
   */
  int single_support_timesteps = 10;

  /**
   * @brief Duration ratio betweep single support and double support
   */
  double double_support_ratio = 1.;

  /**
   * @brief Duration ratio betweep single support and double support
   */
  double startend_double_support_ratio = 1.;

  /**
   * @brief Duratuon [s] of a double support
   */
  double double_support_duration();

  /**
   * @brief Duration [s] of a start/end double support
   */
  double startend_double_support_duration();

  /**
   * @brief Duration [timesteps] of a double support
   */
  int double_support_timesteps();

  /**
   * @brief Duration [timesteps] of a start/end double support
   */
  int startend_double_support_timesteps();

  /**
   * @brief Checks if the walk resulting from those parameters will have double supports
   */
  bool has_double_support();

  /**
   * @brief Kick duration [ms], must be a multiple of dt
   */
  double kick_duration = 1.;

  /**
   * @brief Planning horizon for the CoM trajectory
   */
  int planned_timesteps = 100;

  /**
   * @brief Number of timesteps between each replan.
   * Support phases have to last longer than [replan_frequency * dt] or their duration has to be equal to 0
   */
  int replan_timesteps = 10;

  /**
   * @brief Margin for the ZMP to live in the support polygon [m]
   */
  double zmp_margin = 0.025;

  /**
   * @brief How height the feet are rising while walking [m]
   */
  double walk_foot_height = 0.05;

  /**
   * @brief ratio of time spent at foot height during the step
   */
  double walk_foot_rise_ratio = 0.2;

  /**
   * @brief CoM height while walking [m]
   */
  double walk_com_height = 0.4;

  /**
   * @brief Trunk pitch while walking [rad]
   */
  double walk_trunk_pitch = 0.0;

  /**
   * @brief How much the foot tilts during the walk [rad]
   */
  double walk_foot_tilt = 0.2;

  /**
   * @brief Maximum step (forward)
   */
  double walk_max_dx_forward = 0.08;

  /**
   * @brief Maximum step (backward)
   */
  double walk_max_dx_backward = 0.03;

  /**
   * @brief Maximum step (lateral)
   */
  double walk_max_dy = 0.04;

  /**
   * @brief Maximum step (yaw)
   */
  double walk_max_dtheta = 0.35;

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
  double feet_spacing = 0.15;

  /**
   * @brief Foot width [m]
   */
  double foot_width = 0.1;

  /**
   * @brief Foot length [m]
   */
  double foot_length = 0.15;

  /**
   * @brief Target offset for the ZMP x reference trajectory in the foot frame [m]
   */
  double foot_zmp_target_x = 0.0;

  /**
   * @brief Target offset for the ZMP x reference trajectory in the foot frame, positive is "outward" [m]
   */
  double foot_zmp_target_y = 0.0;

  /**
   * @brief Kick tolerance distance [m]
   */
  double kick_tolerance_distance = 0.03;

  /**
   * @brief Kick tolerance orientation [rad]
   */
  double kick_tolerance_orientation = 0.06;

  /**
   * Natural frequency of the Linear Inverted Pendulum (LIP) model used in the walk
   */
  double omega();

  /**
   * @brief Applies the ellipsoid clipping to a given step size (dx, dy, dtheta)
   */
  Eigen::Vector3d ellipsoid_clip(Eigen::Vector3d step);
};
}  // namespace placo