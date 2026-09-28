#include "placo/kinematics/relative_orientation_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
RelativeOrientationTask::RelativeOrientationTask(model::RobotWrapper::FrameIndex frame_a,
                                                 model::RobotWrapper::FrameIndex frame_b, Eigen::Matrix3d R_a_b)
  : frame_a(frame_a), frame_b(frame_b), R_a_b(R_a_b)
{
}

void RelativeOrientationTask::support()
{
  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a), solver->robot.frame_support(frame_b),
                                      columns);
}

void RelativeOrientationTask::fill()
{
  Eigen::Affine3d T_world_a = solver->robot.get_T_world_frame(frame_a);
  Eigen::Affine3d T_world_b = solver->robot.get_T_world_frame(frame_b);
  Eigen::Affine3d T_a_b = T_world_a.inverse() * T_world_b;

  Eigen::Vector3d error = pinocchio::log3(R_a_b * T_a_b.linear().transpose());

  solver->robot.compact_frame_jacobian(frame_a, pinocchio::WORLD, columns, J_a);
  solver->robot.compact_frame_jacobian(frame_b, pinocchio::WORLD, columns, J_b);
  Eigen::Matrix<double, 3, Eigen::Dynamic> J_ab =
      T_world_a.linear().transpose() * (J_b.bottomRows(3) - J_a.bottomRows(3));

  A.resize(mask.rows(), columns.size());
  b.resize(mask.rows(), 1);
  mask.apply(J_ab, A);
  mask.apply(error, b);
}

std::string RelativeOrientationTask::type_name()
{
  return "relative_orientation";
}

std::string RelativeOrientationTask::error_unit()
{
  return "rad";
}
}  // namespace placo::kinematics