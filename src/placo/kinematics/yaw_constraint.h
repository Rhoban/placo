#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/kinematics/constraint.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief A yaw constraint is a constraint where the x-axis of frame b should have a yaw withing +- angle_max in frame a
 */
class YawConstraint : public Constraint
{
public:
  /**
   * @brief Constrains the yaw of frame b in frame a, such that the x-axis of frame b should remain with +- angle_max
   * @param frame_a
   * @param frame_b
   * @param angle_max
   */
  YawConstraint(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b, double angle_max);

  /**
   * @brief Frame A
   */
  model::RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  model::RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Maximum angle allowable by the yaw constraint (yaw is taken for x-axis of b around z-axis in a)
   */
  double angle_max;

  virtual void add_constraint(placo::problem::Problem& problem) override;
};
}  // namespace placo::kinematics