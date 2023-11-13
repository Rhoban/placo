#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct PositionTask : public Task
{
  /**
   * @brief See \ref KinematicsSolver::add_position_task
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

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  /**
   * @brief Mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::kinematics