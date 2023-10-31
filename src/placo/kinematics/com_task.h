#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct CoMTask : public Task
{
  CoMTask(Eigen::Vector3d target_world);

  Eigen::Vector3d target_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  AxisesMask mask;
};
}  // namespace placo::kinematics