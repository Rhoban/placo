#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct RelativePositionTask : public Task
{
  /**
   * @brief See \ref KinematicsSolver::add_relative_position_task
   */
  RelativePositionTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b, Eigen::Vector3d target);

  /**
   * @brief Frame A
   */
  RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Target position of B in A
   */
  Eigen::Vector3d target;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::kinematics