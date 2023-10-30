#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct GearTask : public Task
{
  struct Gear
  {
    int source;
    double ratio;
  };

  GearTask();

  std::map<int, Gear> gears;

  void set_gear(std::string target, std::string source, double ratio);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics