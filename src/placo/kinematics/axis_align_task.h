#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct AxisAlignTask : public Task
{
  AxisAlignTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d axis_frame,
                Eigen::Vector3d targetAxis_world);

  /**
   * @brief Target frame
   */
  model::RobotWrapper::FrameIndex frame_index;

  /**
   * @brief Axis in the frame
   */
  Eigen::Vector3d axis_frame;

  /**
   * @brief Target axis in the world
   */
  Eigen::Vector3d targetAxis_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics