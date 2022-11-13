#include "placo/control/relative_pose_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
RelativePoseTask::RelativePoseTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b,
                                   Eigen::Affine3d T_a_b)
  : frame_a(frame_a), frame_b(frame_b), T_a_b(T_a_b)
{
}

void RelativePoseTask::update()
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);
  auto T_a_b_current = T_world_a.inverse() * T_world_b;

  // (T_a_b* T_a_b^{-1}) T_a_b = T_a_b*
  // |-----------------|
  //          | This part is the world error that "correct" the transformatio
  Eigen::VectorXd error = pinocchio::log6((T_a_b * T_a_b_current.inverse()).matrix()).toVector();

  auto J_a = solver->robot.frame_jacobian(frame_a, pinocchio::WORLD);
  auto J_b = solver->robot.frame_jacobian(frame_b, pinocchio::WORLD);
  A = pinocchio::SE3(T_world_a.inverse().matrix()).toActionMatrix() * (J_b - J_a);
  b = error;
}

std::string RelativePoseTask::type_name()
{
  return "relative_pose";
}

std::string RelativePoseTask::error_unit()
{
  return "twist-norm";
}
}  // namespace placo