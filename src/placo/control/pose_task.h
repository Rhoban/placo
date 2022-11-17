#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct PoseTask : public Task
{
  PoseTask(RobotWrapper::FrameIndex frame_index, Eigen::Affine3d T_world_frame);

  RobotWrapper::FrameIndex frame_index;
  Eigen::Affine3d T_world_frame;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo