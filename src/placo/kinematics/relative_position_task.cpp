#include "placo/kinematics/relative_position_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
RelativePositionTask::RelativePositionTask(model::RobotWrapper::FrameIndex frame_a,
                                           model::RobotWrapper::FrameIndex frame_b, Eigen::Vector3d target)
  : frame_a(frame_a), frame_b(frame_b), target(target)
{
}

void RelativePositionTask::update()
{
  Eigen::Affine3d T_world_a = solver->robot.get_T_world_frame(frame_a);
  Eigen::Affine3d T_world_b = solver->robot.get_T_world_frame(frame_b);
  Eigen::Affine3d T_a_b = T_world_a.inverse() * T_world_b;

  A = mask.apply(solver->robot.relative_position_jacobian(frame_a, frame_b));
  b = mask.apply(target - T_a_b.translation());
}

std::string RelativePositionTask::type_name()
{
  return "relative_position";
}

std::string RelativePositionTask::error_unit()
{
  return "m";
}

}  // namespace placo::kinematics