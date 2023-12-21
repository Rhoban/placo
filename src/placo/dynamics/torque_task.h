#pragma once

#include <map>
#include <string>
#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class TorqueTask : public Task
{
public:
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_tau_task
   */
  TorqueTask();

  /**
   * @brief Target torques
   */
  std::map<std::string, double> torques;

  /**
   * @brief Sets the target for a given joint
   * @param joint joint name
   * @param torque target torque
   */
  void set_torque(std::string joint, double torque);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics