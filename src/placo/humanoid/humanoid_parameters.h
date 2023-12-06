#pragma once

#include <Eigen/Dense>
#include "placo/humanoid/humanoid_robot.h"

namespace placo::humanoid
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
   * @brief SSP duration [s]
   */
  double single_support_duration = 1.;

  /**
   * @brief Number of timesteps for one single support
   */
  int single_support_timesteps = 10;

  /**
   * @brief Duration ratio between single support and double support
   */
  double double_support_ratio = 1.;

  /**
   * @brief Duration ratio between single support and double support
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
   * @brief Target CoM height while walking [m]
   */
  double walk_com_height = 0.35;

  /**
   * @brief Trunk pitch while walking [rad]
   */
  double walk_trunk_pitch = 0.0;

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
   * @brief How much we need to space the feet per dtheta [m/rad]
   */
  double walk_dtheta_spacing = 0.05;

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
   * @brief Weight for ZMP reference in the solver
   */
  double zmp_reference_weight = 1e-1;

  // Kick parameters
  double kicking_foot_height = 0.05;
  double kick_zmp_target_x = -0.01;
  double kick_zmp_target_y = -0.01;
  double kick_amplitude = 0.13;

  // Timings
  double kick_ratio_up = 1.;
  double kick_ratio_shot = 1.;
  double kick_ratio_neutral = .8;
  double kick_ratio_down = .2;

  double kick_up_duration();
  double kick_shot_duration();
  double kick_neutral_duration();
  double kick_down_duration();

  /**
   * @brief Duration ratio between single support and kick support
   */
  double kick_support_ratio();

  /**
   * @brief Duration [s] of a kick support
   */
  double kick_support_duration();

  /**
   * @brief Duration [timesteps] of a kick support
   */
  int kick_support_timesteps();

  /**
   * @brief Applies the ellipsoid clipping to a given step size (dx, dy, dtheta)
   */
  Eigen::Vector3d ellipsoid_clip(Eigen::Vector3d step);

  /**
   * @brief Frames for opposite and neutral positions
   */
  Eigen::Affine3d opposite_frame(HumanoidRobot::Side side, Eigen::Affine3d T_world_foot, double d_x = 0.,
                                 double d_y = 0., double d_theta = 0.);
  Eigen::Affine3d neutral_frame(HumanoidRobot::Side side, Eigen::Affine3d T_world_foot, double d_x = 0.,
                                double d_y = 0., double d_theta = 0.);

protected:
  /**
   * @brief How many timesteps are required for a given ratio ?
   */
  int ratio_timesteps(double ratio);

  /**
   * @brief What is the (rounded to timesteps) duration for a given ratio ?
   */
  double ratio_duration(double ratio);
};
}  // namespace placo::humanoid