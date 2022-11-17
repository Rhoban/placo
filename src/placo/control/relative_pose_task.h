#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct RelativePoseTask : public Task
{
  RelativePoseTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b, Eigen::Affine3d T_a_b);

  RobotWrapper::FrameIndex frame_a;
  RobotWrapper::FrameIndex frame_b;
  Eigen::Affine3d T_a_b;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo