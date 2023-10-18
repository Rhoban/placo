#include "placo/kinematics/relative_frame_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
RelativeFrameTask::RelativeFrameTask(RelativePositionTask& position, RelativeOrientationTask& orientation)
  : position(position), orientation(orientation)
{
}

void RelativeFrameTask::configure(std::string name, std::string priority, double position_weight,
                                  double orientation_weight)
{
  position.configure(name + "_position", priority, position_weight);
  orientation.configure(name + "_orientation", priority, orientation_weight);
}

Eigen::Affine3d RelativeFrameTask::get_T_a_b() const
{
  Eigen::Affine3d T;
  T.translation() = position.target;
  T.linear() = orientation.R_a_b;

  return T;
}

void RelativeFrameTask::set_T_a_b(Eigen::Affine3d T_a_b)
{
  position.target = T_a_b.translation();
  orientation.R_a_b = T_a_b.linear();
}
}  // namespace placo::kinematics