#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct RegularizationTask : public Task
{
  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo