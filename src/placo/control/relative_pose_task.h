#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct RelativePoseTask : public Task
{
  RelativePoseTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Affine3d T_a_b);

  MobileRobot::FrameIndex frame_a;
  MobileRobot::FrameIndex frame_b;
  Eigen::Affine3d T_a_b;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo