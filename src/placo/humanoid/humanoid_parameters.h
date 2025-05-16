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
   * @brief Timestep duration for planning [s]
   */
  double dt();

  /**
   * @brief Single support duration [s]
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
   * @brief Duration ratio between single support and start/end double support
   */
  double startend_double_support_ratio = 1.;

  /**
   * @brief Default duration [s] of a double support
   */
  double double_support_duration();

  /**
   * @brief Default duration [s] of a start/end double support
   */
  double startend_double_support_duration();

  /**
   * @brief Default number of timesteps for one double support
   */
  int double_support_timesteps();

  /**
   * @brief Default number of timesteps for one start/end double support
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

  // TODO: use this operational space with the FootstepsPlanners
  // Operational space of the flying foot (half-ellipse)
  double op_space_y_offset = 0.1;
  double op_space_x_radius = 0.35;
  double op_space_y_radius = 0.3;

  // Operational space of the flying foot (polygon)
  std::vector<Eigen::Vector2d> op_space_polygon;

  // DCM offset bounds (polygon)
  std::vector<Eigen::Vector2d> dcm_offset_polygon;

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
};
}  // namespace placo::humanoid