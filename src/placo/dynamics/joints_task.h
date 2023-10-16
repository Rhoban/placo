#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/control/axises_mask.h"

namespace placo
{
namespace dynamics
{
class JointsTask : public Task
{
public:
  JointsTask();

  void set_joint(std::string joint, double target);

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  std::map<std::string, double> joints;
};
}  // namespace dynamics
}  // namespace placo