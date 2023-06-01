#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct PositionTask : public Task
{
  PositionTask(RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world);

  RobotWrapper::FrameIndex frame_index;
  Eigen::Vector3d target_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  enum Mask
  {
    MaskX = 1,
    MaskY = 2,
    MaskZ = 4
  };

  int mask = MaskX | MaskY | MaskZ;
};
}  // namespace placo