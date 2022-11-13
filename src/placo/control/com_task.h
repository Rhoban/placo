#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct CoMTask : public Task
{
  CoMTask(Eigen::Vector3d target_world);

  Eigen::Vector3d target_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo