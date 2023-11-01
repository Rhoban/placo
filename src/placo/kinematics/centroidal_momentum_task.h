#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct CentroidalMomentumTask : public Task
{
  /**
   * @brief See \ref KinematicsSolver::add_centroidal_momentum_task
   */
  CentroidalMomentumTask(Eigen::Vector3d L_world);

  std::set<int> active_axises;

  /**
   * @brief Target centroidal angular momentum in the world
   */
  Eigen::Vector3d L_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  /**
   * @brief Axises mask
   */
  tools::AxisesMask mask;
};
}  // namespace placo::kinematics