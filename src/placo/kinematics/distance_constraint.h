#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/kinematics/constraint.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief Constraints the distance betweek two points in the robot
 */
class DistanceConstraint : public Constraint
{
public:
  /**
   * @brief Constraints the distance betweek two points in the robot
   * @param frame_a
   * @param frame_b
   * @param distance_max
   */
  DistanceConstraint(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b,
                     double distance_max);

  /**
   * @brief Frame A
   */
  model::RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  model::RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Maximum distance allowed between frame A and frame B
   */
  double distance_max;

  virtual void add_constraint(placo::problem::Problem& problem) override;
};
}  // namespace placo::kinematics