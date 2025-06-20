#pragma once

#include <Eigen/Dense>
#include "placo/kinematics/kinematics_solver.h"
#include "placo/tools/cubic_spline.h"
#include "placo/humanoid/humanoid_parameters.h"
#include "placo/humanoid/footsteps_planner_repetitive.h"

namespace placo::humanoid
{
class DummyWalk
{
public:
  DummyWalk(model::RobotWrapper& robot, humanoid::HumanoidParameters& parameters);

  /**
   * @brief Reset the robot with a given support
   * @param support_left whether the first support is left
   */
  void reset(bool support_left = false);

  /**
   * @brief Produce the next step, change support foot
   * @param dx dx
   * @param dy dy
   * @param dtheta dtheta
   */
  void next_step(double dx, double dy, double dtheta);

  /**
   * @brief Updates the internal IK
   * @param t phase in step from 0 to 1
   */
  void update(double t);

  /**
   * @brief Update the support to a given world pose
   * @param T_world_support
   */
  void update_T_world_support(Eigen::Affine3d T_world_support);

  /**
   * @brief Robot wrapper
   */
  model::RobotWrapper& robot;

  /**
   * @brief Humanoid parameters
   */
  humanoid::HumanoidParameters& parameters;

  /**
   * @brief Kinematics solver
   */
  kinematics::KinematicsSolver solver;

  /**
   * @brief Trunk x-offset [m]
   */
  double trunk_x_offset = 0.05;

  /**
   * @brief Whether the current support is left or right
   */
  bool support_left;

  /**
   * @brief Cubic splines for the lift trajectory
   */
  tools::CubicSpline lift_spline;

  /**
   * @brief left foot in world, at begining of current step
   */
  Eigen::Affine3d T_world_left;

  /**
   * @brief right foot in world, at begining of current step
   */
  Eigen::Affine3d T_world_right;

  /**
   * @brief Target for the current flying foot (given by support_left)
   */
  Eigen::Affine3d T_world_next;

  // Internal tasks
  kinematics::FrameTask left_foot_task;
  kinematics::FrameTask right_foot_task;
  kinematics::FrameTask trunk_task;

  /**
   * @brief Last requested step dx
   */
  double dx = 0.0;

  /**
   * @brief Last requested step d-
   */
  double dy = 0.0;

  /**
   * @brief Last requested step dtheta
   */
  double dtheta = 0.0;

protected:
  void compute_next_support(double dx, double dy, double dtheta);
  Eigen::Affine3d translation(double x, double y, double z) const;

  /**
   * @brief Internal footsteps planner
   */
  humanoid::FootstepsPlannerRepetitive footsteps_planner;

  /**
   * @brief Solve the internal kinematics problem
   */
  void solve();
};
}  // namespace placo::humanoid