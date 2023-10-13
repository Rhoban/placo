#pragma once

#include "placo/control/task.h"
#include "placo/control/axises_mask.h"

namespace placo
{
class KinematicsSolver;
struct CoMBoundTask : public Task
{
  CoMBoundTask(double bound, int dir);

  double bound;
  int dir;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo