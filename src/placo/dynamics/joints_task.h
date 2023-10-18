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
  JointsTask();

  std::map<std::string, double> joints;
  std::map<std::string, double> djoints;

  void set_joint(std::string joint, double target, double velocity = 0.);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace placo::dynamics