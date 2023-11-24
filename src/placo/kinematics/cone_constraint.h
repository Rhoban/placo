#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/kinematics/constraint.h"

namespace placo::kinematics
{
class KinematicsSolver;
class ConeConstraint : public Constraint
{
public:
  ConeConstraint(model::RobotWrapper::FrameIndex frame, double alpha_max,
                 Eigen::Affine3d T_world_cone = Eigen::Affine3d::Identity());

  /**
   * @brief Frame being used (its z axis has to remain within an angle of alpha_max with the z-axis of T_world_cone)
   */
  model::RobotWrapper::FrameIndex frame;

  /**
   * @brief Maximum angle allowable by the cone constraint (between the frame z axis and T_world_cone z axis)
   */
  double alpha_max;

  /**
   * @brief The (inertial) frame in which the cone is defined
   */
  Eigen::Affine3d T_world_cone;

  /**
   * @brief Number of slices used to discretize the cone
   */
  int N = 8;

  /**
   * @brief Range of the cone discretization (in radians). The cone is discretized in [-range, range] around
   * the current orientation.
   */
  double range = 0.25;

  virtual void add_constraint(placo::problem::Problem& problem) override;
};
}  // namespace placo::kinematics