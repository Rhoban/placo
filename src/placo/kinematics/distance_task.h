#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct DistanceTask : public Task
{
  /**
   * @brief see \ref KinematicsSolver::add_distance_task
   */
  DistanceTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b, double distance);

  /**
   * @brief Frame A
   */
  RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Target distance between A and B
   */
  double distance;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics