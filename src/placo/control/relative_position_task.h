#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct RelativePositionTask : public Task
{
  RelativePositionTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Vector3d target);

  MobileRobot::FrameIndex frame_a;
  MobileRobot::FrameIndex frame_b;
  Eigen::Vector3d target;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo