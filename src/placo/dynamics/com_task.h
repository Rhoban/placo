#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
/**
 * @brief CoM task (expressed in the world)
 */
class CoMTask : public Task
{
public:
  /**
   * @brief CoM task
   * @param target_world target for the world CoM
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

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  /**
   * @brief Axises mask
   */
  AxisesMask mask;
};
}  // namespace placo::dynamics