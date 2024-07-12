#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class PositionTask : public Task
{
public:
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_position_task
   */
  PositionTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world);

  /**
   * @brief Frame
   */
  model::RobotWrapper::FrameIndex frame_index;

  /**
   * @brief Target position in the world
   */
  Eigen::Vector3d target_world;

  /**
   * @brief Target velocity in the world
   */
  Eigen::Vector3d dtarget_world = Eigen::Vector3d::Zero();

  /**
   * @brief Target acceleration in the world
   */
  Eigen::Vector3d ddtarget_world = Eigen::Vector3d::Zero();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::dynamics