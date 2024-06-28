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
  struct TargetTau
  {
    /**
     * @brief Torque to apply
     */
    double torque = 0.0;

    /**
     * @brief Proportional gain
     */
    double kp = 0.0;

    /**
     * @brief Derivative gain (damping)
     */
    double kd = 0.0;
  };

  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_tau_task
   */
  TorqueTask();

  /**
   * @brief Target torques
   */
  std::map<std::string, TargetTau> torques;

  /**
   * @brief Sets the target for a given joint
   * @param joint joint name
   * @param torque target torque
   * @param kp proportional gain (optional)
   * @param kd derivative gain (optional)
   */
  void set_torque(std::string joint, double torque, double kp = 0.0, double kd = 0.0);

  /**
   * @brief Removes a joint from this task
   * @param joint joint namle
   */
  void reset_torque(std::string joint);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics