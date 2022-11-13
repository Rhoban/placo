#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;

struct RelativeOrientationTask : public Task
{
  RelativeOrientationTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Matrix3d R_a_b);

  MobileRobot::FrameIndex frame_a;
  MobileRobot::FrameIndex frame_b;
  Eigen::Matrix3d R_a_b;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo