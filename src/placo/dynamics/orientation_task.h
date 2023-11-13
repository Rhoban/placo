#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class OrientationTask : public Task
{
public:
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_orientation_task
   */
  OrientationTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d target_world);

  /**
   * @brief frame
   */
  model::RobotWrapper::FrameIndex frame_index;

  /**
   * @brief Target orientation
   */
  Eigen::Matrix3d R_world_frame;

  /**
   * @brief Target angular velocity
   */
  Eigen::Vector3d omega_world = Eigen::Vector3d::Zero();

  /**
   * @brief Target angular acceleration
   */
  Eigen::Vector3d domega_world = Eigen::Vector3d::Zero();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::dynamics