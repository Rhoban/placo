#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class CoMTask : public Task
{
public:
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_com_task
   */
  CoMTask(Eigen::Vector3d target_world);

  /**
   * @brief Target to reach in world frame
   */
  Eigen::Vector3d target_world;

  /**
   * @brief Target velocity to reach in robot frame
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
   * @brief Axises mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::dynamics