#pragma once

#include "placo/dynamics/relative_position_task.h"
#include "placo/dynamics/relative_orientation_task.h"
#include "placo/dynamics/orientation_task.h"

namespace placo::dynamics
{
struct RelativeFrameTask
{
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_relative_frame_task
   */
  RelativeFrameTask();

  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_relative_frame_task
   */
  RelativeFrameTask(RelativePositionTask* position, RelativeOrientationTask* orientation);

  /**
   * @brief Configures the relative frame task
   * @param name task name
   * @param priority task priority
   * @param position_weight weight for the position task
   * @param orientation_weight weight for the orientation task
   */
  void configure(std::string name, std::string priority = "soft", double position_weight = 1.0,
                 double orientation_weight = 1.0);

  /**
   * @brief Position task
   */
  RelativePositionTask* position;

  /**
   * @brief Orientation task
   */
  RelativeOrientationTask* orientation;

  /**
   * @brief Retrieve the relative frame from tasks
   * @return transformation
   */
  Eigen::Affine3d get_T_a_b() const;

  /**
   * @brief Set the relative frame target for tasks
   * @param T_a_b transformation
   */
  void set_T_a_b(Eigen::Affine3d T_a_b);
};
}  // namespace placo::dynamics