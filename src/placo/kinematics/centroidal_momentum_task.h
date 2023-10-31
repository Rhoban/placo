#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;

struct CentroidalMomentumTask : public Task
{
  CentroidalMomentumTask(Eigen::Vector3d L_world);

  std::set<int> active_axises;

  /**
   * @brief Can be used to "mask" (disable) an axis. For instance, if you mask axis 2, the task will not be imposed
   *        to centroidal momentum around z axis
   * @param axis 0, 1 or 2 (will be x, y, or z in the world)
   */
  void mask_axis(int axis);

  Eigen::Vector3d L_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics