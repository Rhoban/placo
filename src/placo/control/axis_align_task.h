#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct AxisAlignTask : public Task
{
  AxisAlignTask(RobotWrapper::FrameIndex frame_index, Eigen::Vector3d axis_frame, Eigen::Vector3d targetAxis_world);

  RobotWrapper::FrameIndex frame_index;
  Eigen::Vector3d axis_frame;
  Eigen::Vector3d targetAxis_world;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo