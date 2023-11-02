#pragma once

#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"

namespace placo::dynamics
{
struct FrameTask
{
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_frame_task
   */
  FrameTask();

  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_frame_task
   */
  FrameTask(PositionTask* position, OrientationTask* orientation);

  /**
   * @brief Configures the frame task
   * @param name task name
   * @param priority task priority
   * @param position_weight weight for the position task
   * @param orientation_weight weight for the orientation task
   */
  void configure(std::string name, std::string priority = "soft", double position_weight = 1.0,
                 double orientation_weight = 1.0);

  PositionTask* position;
  OrientationTask* orientation;

  Eigen::Affine3d get_T_world_frame() const;
  void set_T_world_frame(Eigen::Affine3d T_world_frame);
};
}  // namespace placo::dynamics