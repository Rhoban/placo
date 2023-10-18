#pragma once

#include "placo/kinematics/task.h"
#include "placo/kinematics/relative_position_task.h"
#include "placo/kinematics/relative_orientation_task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct RelativeFrameTask
{
  RelativeFrameTask(RelativePositionTask& position, RelativeOrientationTask& orientation);

  void configure(std::string name, std::string priority = "soft", double position_weight = 1.0,
                 double orientation_weight = 1.0);

  RelativePositionTask& position;
  RelativeOrientationTask& orientation;

  Eigen::Affine3d get_T_a_b() const;
  void set_T_a_b(Eigen::Affine3d T_world_frame);
};
}  // namespace placo::kinematics