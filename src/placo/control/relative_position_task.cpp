#include "placo/control/relative_position_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
RelativePositionTask::RelativePositionTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b,
                                           Eigen::Vector3d target)
  : frame_a(frame_a), frame_b(frame_b), target(target)
{
}

void RelativePositionTask::update()
{
  auto T_world_a = solver->robot->get_T_world_frame(frame_a);
  auto T_world_b = solver->robot->get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  Eigen::Vector3d error = target - T_a_b.translation();

  auto J_a = solver->robot->frame_jacobian(frame_a, pinocchio::WORLD);
  auto J_b = solver->robot->frame_jacobian(frame_b, pinocchio::WORLD);

  A = (pinocchio::SE3(T_world_a.inverse().matrix()).toActionMatrix() * (J_b - J_a)).block(0, 0, 3, solver->N);
  b = error;
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