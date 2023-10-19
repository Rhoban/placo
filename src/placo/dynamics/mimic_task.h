#pragma once

#include <map>
#include <string>
#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class MimicTask : public Task
{
public:
  struct Mimic
  {
    int source;
    double ratio;
  };
  MimicTask();

  std::map<int, Mimic> mimics;

  void set_mimic(std::string target, std::string source, double ratio);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics