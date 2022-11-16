#include "placo/control/pose_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
PoseTask::PoseTask(MobileRobot::FrameIndex frame_index, Eigen::Affine3d T_world_frame)
  : frame_index(frame_index), T_world_frame(T_world_frame)
{
}

void PoseTask::update()
{
  auto T_world_frame_current = solver->robot->get_T_world_frame(frame_index);

  // (T_frame T_current^{-1}) T_current = T_frame
  // |-----------------------|
  //            | This part is the world error that "correct" the transformatio
  Eigen::VectorXd error = pinocchio::log6((T_world_frame * T_world_frame_current.inverse()).matrix()).toVector();

  A = solver->robot->frame_jacobian(frame_index, pinocchio::WORLD);
  b = error;
}

std::string PoseTask::type_name()
{
  return "pose";
}

std::string PoseTask::error_unit()
{
  return "twist-norm";
}
}  // namespace placo