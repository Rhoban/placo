#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct JointTask : public Task
{
  JointTask(std::string joint, double target);

  std::string joint;
  double target;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo