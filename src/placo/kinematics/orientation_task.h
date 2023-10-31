#pragma once

#include "placo/kinematics/task.h"
#include "placo/tools/axises_mask.h"

namespace placo::kinematics
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

  AxisesMask mask;
};
}  // namespace placo::kinematics