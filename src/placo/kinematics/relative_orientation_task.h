#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
{
class KinematicsSolver;

struct RelativeOrientationTask : public Task
{
  RelativeOrientationTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b, Eigen::Matrix3d R_a_b);

  RobotWrapper::FrameIndex frame_a;
  RobotWrapper::FrameIndex frame_b;
  Eigen::Matrix3d R_a_b;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  AxisesMask mask;
};
}  // namespace placo::kinematics