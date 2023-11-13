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
  RelativeOrientationTask(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b,
                          Eigen::Matrix3d R_a_b);

  /**
   * @brief Frame A
   */
  model::RobotWrapper::FrameIndex frame_a;

  /**
   * @brief Frame B
   */
  model::RobotWrapper::FrameIndex frame_b;

  /**
   * @brief Target relative orientation of b in a
   */
  Eigen::Matrix3d R_a_b;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::kinematics