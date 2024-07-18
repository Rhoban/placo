#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct JointsTask : public Task
{
  /**
   * @brief see \ref KinematicsSolver::add_joints_task
   */
  JointsTask();

  /**
   * @brief Joint names to target values mapping
   */
  std::map<std::string, double> joints;

  /**
   * @brief Sets a joint target
   * @param joint joint
   * @param target target value
   */
  void set_joint(std::string joint, double target);

  /**
   * @brief Returns the target value of a joint
   * @param joint joint
   * @return target value
   */
  double get_joint(std::string joint);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics