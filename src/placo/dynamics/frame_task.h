#pragma once

#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"

namespace placo
{
namespace dynamics
{
struct FrameTask
{
  FrameTask();
  FrameTask(PositionTask* position, OrientationTask* orientation);

  void configure(std::string name, std::string priority = "soft", double position_weight = 1.0,
                 double orientation_weight = 1.0);

  PositionTask* position;
  OrientationTask* orientation;

  Eigen::Affine3d get_T_world_frame() const;
  void set_T_world_frame(Eigen::Affine3d T_world_frame);
};
};  // namespace dynamics
}  // namespace placo