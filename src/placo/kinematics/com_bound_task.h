#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
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
}  // namespace placo::kinematics