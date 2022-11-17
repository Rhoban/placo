#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;

struct OrientationTask : public Task
{
  OrientationTask(RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);

  RobotWrapper::FrameIndex frame_index;
  Eigen::Matrix3d R_world_frame;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo