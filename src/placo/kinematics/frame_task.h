#pragma once

#include "placo/kinematics/position_task.h"
#include "placo/kinematics/orientation_task.h"

namespace placo::kinematics
{
struct FrameTask
{
  /**
   * @brief see \ref KinematicsSolver::add_frame_task
   */
  FrameTask();

  /**
   * @brief see \ref KinematicsSolver::add_frame_task
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

  /**
   * @brief Position task
   */
  PositionTask* position;

  /**
   * @brief Orientation task
   */
  OrientationTask* orientation;

  /**
   * @brief Retrieve the T_world_frame from tasks
   * @return transformation
   */
  Eigen::Affine3d get_T_world_frame() const;

  /**
   * @brief Sets the tasks target from given transformation
   * @param T_world_frame transformation
   */
  void set_T_world_frame(Eigen::Affine3d T_world_frame);
};
}  // namespace placo::kinematics