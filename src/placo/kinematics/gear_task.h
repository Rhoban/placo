#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct GearTask : public Task
{
  /**
   * @brief A gear entry configures a joint-mimic with ratio
   */
  struct Gear
  {
    /**
     * @brief Target joint
     */
    int target;

    /**
     * @brief Source joint
     */
    int source;

    /**
     * @brief Ratio
     */
    double ratio;
  };

  /**
   * @brief see \ref KinematicsSolver::add_gear_task
   */
  GearTask();

  /**
   * @brief Gear settings
   */
  std::map<int, Gear> gears;

  /**
   * @brief Sets a gear constraint
   * @param target target joint
   * @param source source joint
   * @param ratio ratio
   */
  void set_gear(std::string target, std::string source, double ratio);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics