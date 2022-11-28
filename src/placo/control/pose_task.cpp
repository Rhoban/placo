#include "placo/control/pose_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
PoseTask::PoseTask(RobotWrapper::FrameIndex frame_index, Eigen::Affine3d T_world_frame)
  : frame_index(frame_index), T_world_frame(T_world_frame)
{
}

void PoseTask::update()
{
  auto T_world_frame_current = solver->robot->get_T_world_frame(frame_index);
  auto M = (T_world_frame_current.inverse() * T_world_frame).matrix();

  // (T_frame T_current^{-1}) T_current = T_frame
  // |-----------------------|
  //            | This part is the world error that "correct" the transformation
  Eigen::VectorXd error = pinocchio::log6(M).toVector();
  Eigen::Matrix<double, 6, 6> Jlog;
  pinocchio::Jlog6(pinocchio::SE3(M), Jlog);

  A = Jlog * solver->robot->frame_jacobian(frame_index, pinocchio::LOCAL);
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