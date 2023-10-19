#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct MimicTask : public Task
{
  struct Mimic
  {
    int source;
    double ratio;
  };

  MimicTask();

  std::map<int, Mimic> mimics;

  void set_mimic(std::string target, std::string source, double ratio);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics