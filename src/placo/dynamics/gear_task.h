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
  struct Gear
  {
    int source;
    double ratio;
  };
  GearTask();

  std::map<int, Gear> gears;

  void set_gear(std::string target, std::string source, double ratio);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics