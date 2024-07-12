#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class RelativePositionTask : public Task
{
public:
  RelativePositionTask(model::RobotWrapper::FrameIndex frame_a_index, model::RobotWrapper::FrameIndex frame_b_index,
                       Eigen::Vector3d target_world);

  /**
   * @brief Frame A
   */
  model::RobotWrapper::FrameIndex frame_a_index;

  /**
   * @brief Frame B
   */
  model::RobotWrapper::FrameIndex frame_b_index;

  /**
   * @brief Target relative position
   */
  Eigen::Vector3d target;

  /**
   * @brief Target relative velocity
   */
  Eigen::Vector3d dtarget = Eigen::Vector3d::Zero();

  /**
   * @brief Target relative acceleration
   */
  Eigen::Vector3d ddtarget = Eigen::Vector3d::Zero();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::dynamics