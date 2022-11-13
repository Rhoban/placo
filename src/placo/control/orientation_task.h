#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;

struct OrientationTask : public Task
{
  OrientationTask(MobileRobot::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);

  MobileRobot::FrameIndex frame_index;
  Eigen::Matrix3d R_world_frame;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo