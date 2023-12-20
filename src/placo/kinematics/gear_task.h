#pragma once

#include <vector>
#include <map>
#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct GearTask : public Task
{
  /**
   * @brief see \ref KinematicsSolver::add_gear_task
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

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics