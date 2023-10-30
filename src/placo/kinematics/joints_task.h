#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct JointsTask : public Task
{
  JointsTask();

  std::map<std::string, double> joints;

  void set_joint(std::string joint, double target);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics