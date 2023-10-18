#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"

namespace placo::dynamics
{
class OrientationTask : public Task
{
public:
  OrientationTask(RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d target_world);

  RobotWrapper::FrameIndex frame_index;
  Eigen::Matrix3d R_world_frame;
  Eigen::Vector3d omega_world = Eigen::Vector3d::Zero();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  AxisesMask mask;
};
}  // namespace placo::dynamics