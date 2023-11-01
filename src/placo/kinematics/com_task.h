#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct CoMTask : public Task
{
  /**
   * @brief See \ref KinematicsSolver::add_com_task
   */
  CoMTask(Eigen::Vector3d target_world);

  /**
   * @brief Target for the CoM in the world
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