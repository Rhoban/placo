#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/control/axises_mask.h"

namespace placo
{
namespace dynamics
{
class OrientationTask : public Task
{
public:
  OrientationTask(RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d target_world);

  RobotWrapper::FrameIndex frame_index;
  Eigen::Matrix3d R_world_frame;

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  AxisesMask mask;

  double kp = 1e3;
};
}  // namespace dynamics
}  // namespace placo