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
   * @brief see \ref placo::dynamics::DynamicSolver::add_gear_task
   */
  GearTask();

  std::map<int, Gear> gears;

  void set_gear(std::string target, std::string source, double ratio);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics