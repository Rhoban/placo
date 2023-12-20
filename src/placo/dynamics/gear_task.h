#pragma once

#include <map>
#include <string>
#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class GearTask : public Task
{
public:
  /**
   * @brief see \ref placo::dynamics::DynamicsSolver::add_gear_task
   */
  GearTask();

  /**
   * @brief Gear settings
   */
  std::map<int, std::map<int, double>> gears;

  /**
   * @brief Sets a gear constraint
   * @param target target joint
   * @param source source joint
   * @param ratio ratio
   */
  void set_gear(std::string target, std::string source, double ratio);

  /**
   * @brief Adds a gear constraint, you can add multiple source for the same target, they
   * will be summed
   * @param target target joint
   * @param source source joint
   * @param ratio ratio
   */
  void add_gear(std::string target, std::string source, double ratio);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics