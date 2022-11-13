#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct PositionTask : public Task
{
  PositionTask(MobileRobot::FrameIndex frame_index, Eigen::Vector3d target_world);

  MobileRobot::FrameIndex frame_index;
  Eigen::Vector3d target_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo