#pragma once

#include "placo/kinematics/task.h"
#include "placo/kinematics/relative_position_task.h"
#include "placo/kinematics/relative_orientation_task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct RelativeFrameTask
{
  /**
   * @brief see \ref KinematicsSolver::add_relative_frame_task
   */
  RelativeFrameTask(RelativePositionTask& position, RelativeOrientationTask& orientation);

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
   * @brief Relative position
   */
  RelativePositionTask& position;

  /**
   * @brief Relative orientation
   */
  RelativeOrientationTask& orientation;

  /**
   * @brief Retrieve the target T_a_b from tasks
   * @return transformation
   */
  Eigen::Affine3d get_T_a_b() const;

  /**
   * @brief Sets the target T_a_b
   * @param T_world_frame transformation
   */
  void set_T_a_b(Eigen::Affine3d T_world_frame);
};
}  // namespace placo::kinematics