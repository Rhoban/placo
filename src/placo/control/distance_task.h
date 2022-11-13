#pragma once

#include "placo/control/task.h"

namespace placo
{
class KinematicsSolver;
struct DistanceTask : public Task
{
  DistanceTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, double distance);

  MobileRobot::FrameIndex frame_a;
  MobileRobot::FrameIndex frame_b;
  double distance;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo