#pragma once

#include <map>
#include <string>
#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/control/axises_mask.h"

namespace placo
{
namespace dynamics
{
class StaticTask : public Task
{
public:
  StaticTask();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;
};
}  // namespace dynamics
}  // namespace placo