#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;

struct RelativeOrientationTask : public Task
{
  /**
   * @brief See \ref KinematicsSolver::add_relative_orientation_task
   */
  RelativeOrientationTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b, Eigen::Matrix3d R_a_b);

  /**
   * @brief Frame A
   */
  RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Relative orientation of b in a
   */
  Eigen::Matrix3d R_a_b;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  /**
   * @brief Mask
   */
  AxisesMask mask;
};
}  // namespace placo::kinematics