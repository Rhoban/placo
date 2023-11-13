#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class RelativeOrientationTask : public Task
{
public:
  RelativeOrientationTask(model::RobotWrapper::FrameIndex frame_a_index, model::RobotWrapper::FrameIndex frame_b_index,
                          Eigen::Matrix3d R_a_b);

  /**
   * @brief Frame A
   */
  model::RobotWrapper::FrameIndex frame_a_index;

  /**
   * @brief Frame B
   */
  model::RobotWrapper::FrameIndex frame_b_index;

  /**
   * @brief Target relative orientation
   */
  Eigen::Matrix3d R_a_b;

  /**
   * @brief Target relative angular velocity
   */
  Eigen::Vector3d omega_a_b = Eigen::Vector3d::Zero();

  /**
   * @brief Target relative angular velocity
   */
  Eigen::Vector3d domega_a_b = Eigen::Vector3d::Zero();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::dynamics