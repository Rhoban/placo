#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct WheelTask : public Task
{
  /**
   * @brief See \ref KinematicsSolver::add_wheel_task
   */
  WheelTask(std::string joint, double radius, bool omniwheel = false);

  /**
   * @brief Frame
   */
  std::string joint;

  /**
   * @brief Wheel radius
   */
  double radius;

  /**
   * @brief Omniwheel (can slide laterally)
   */
  bool omniwheel;

  /**
   * @brief Target position in the world
   */
  Eigen::Affine3d T_world_surface;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics