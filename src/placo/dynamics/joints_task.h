#pragma once

#include <map>
#include <string>
#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class JointsTask : public Task
{
public:
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_gear_task
   */
  JointsTask();

  /**
   * @brief Maps joint to position targets
   */
  std::map<std::string, double> joints;

  /**
   * @brief Maps joints to velocity targets
   */
  std::map<std::string, double> djoints;

  /**
   * @brief Maps joints to acceleration targets
   */
  std::map<std::string, double> ddjoints;

  /**
   * @brief Sets the target for a given joint
   * @param joint joint name
   * @param target target position
   * @param velocity target velocity
   * @param acceleration target acceleration
   */
  void set_joint(std::string joint, double target, double velocity = 0., double acceleration = 0.);

  /**
   * @brief Returns the current target position of a joint
   * @param joint joint name
   * @return current target position
   */
  double get_joint(std::string joint);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics