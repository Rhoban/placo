#pragma once

#include "placo/dynamics/task.h"
#include "placo/model/robot_wrapper.h"
#include "placo/control/axises_mask.h"

namespace placo
{
namespace dynamics
{
class RelativePositionTask : public Task
{
public:
  RelativePositionTask(RobotWrapper::FrameIndex frame_a_index, RobotWrapper::FrameIndex frame_b_index,
                       Eigen::Vector3d target_world);

  RobotWrapper::FrameIndex frame_a_index;
  RobotWrapper::FrameIndex frame_b_index;
  Eigen::Vector3d target;
  Eigen::Vector3d dtarget = Eigen::Vector3d::Zero();

  void update() override;
  std::string type_name() override;
  std::string error_unit() override;

  AxisesMask mask;
};
}  // namespace dynamics
}  // namespace placo