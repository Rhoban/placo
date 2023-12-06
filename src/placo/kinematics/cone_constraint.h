#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/kinematics/constraint.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief A cone constraint is a constraint where the z-axis of frame a and frame b should remaine within a cone of
 * angle angle_max.
 */
class ConeConstraint : public Constraint
{
public:
  /**
   * @brief With a cone constraint, the z-axis of frame a and frame b should remaine within a cone of angle angle_max
   * @param frame_a
   * @param frame_b
   * @param angle_max
   */
  ConeConstraint(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b, double angle_max);

  /**
   * @brief Frame A
   */
  model::RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  model::RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Maximum angle allowable by the cone constraint (between z-axis of frame_a and frame_b)
   */
  double angle_max;

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