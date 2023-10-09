#include "placo/control/relative_position_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
RelativePositionTask::RelativePositionTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b,
                                           Eigen::Vector3d target)
  : frame_a(frame_a), frame_b(frame_b), target(target)
{
}

void RelativePositionTask::update()
{
  auto T_world_a = solver->robot->get_T_world_frame(frame_a);
  auto T_world_b = solver->robot->get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  A = solver->robot->relative_position_jacobian(frame_a, frame_b)(mask.indices, Eigen::placeholders::all);
  b = (target - T_a_b.translation())(mask.indices, Eigen::placeholders::all);
}

std::string RelativePositionTask::type_name()
{
  return "relative_position";
}

std::string RelativePositionTask::error_unit()
{
  return "m";
}

}  // namespace placo