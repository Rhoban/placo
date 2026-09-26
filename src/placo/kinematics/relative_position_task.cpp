#include "placo/kinematics/relative_position_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
RelativePositionTask::RelativePositionTask(model::RobotWrapper::FrameIndex frame_a,
                                           model::RobotWrapper::FrameIndex frame_b, Eigen::Vector3d target)
  : frame_a(frame_a), frame_b(frame_b), target(target)
{
}

void RelativePositionTask::support()
{
  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a), solver->robot.frame_support(frame_b),
                                      columns);
}

void RelativePositionTask::fill()
{
  Eigen::Affine3d T_world_a = solver->robot.get_T_world_frame(frame_a);
  Eigen::Affine3d T_world_b = solver->robot.get_T_world_frame(frame_b);
  Eigen::Affine3d T_a_b = T_world_a.inverse() * T_world_b;
  Eigen::Matrix3d R_a_world = T_world_a.linear().transpose();

  solver->robot.compact_frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_a);
  solver->robot.compact_frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_b);

  // Velocity of b relative to a, expressed in a
  Eigen::Matrix<double, 3, Eigen::Dynamic> J = R_a_world * (J_b.topRows(3) - J_a.topRows(3)) +
                                               pinocchio::skew(T_a_b.translation()) * R_a_world * J_a.bottomRows(3);
  Eigen::Vector3d error = target - T_a_b.translation();

  A.resize(mask.rows(), columns.size());
  b.resize(mask.rows(), 1);
  mask.apply(J, A);
  mask.apply(error, b);
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