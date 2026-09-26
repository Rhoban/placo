#include "placo/dynamics/relative_orientation_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{

RelativeOrientationTask::RelativeOrientationTask(model::RobotWrapper::FrameIndex frame_a_index,
                                                 model::RobotWrapper::FrameIndex frame_b_index, Eigen::Matrix3d R_a_b)
{
  this->frame_a_index = frame_a_index;
  this->frame_b_index = frame_b_index;
  this->R_a_b = R_a_b;
}

void RelativeOrientationTask::support()
{
  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a_index),
                                      solver->robot.frame_support(frame_b_index), columns);
}

void RelativeOrientationTask::fill()
{
  // Computing J and dJ
  pinocchio::ReferenceFrame frame_type = pinocchio::ReferenceFrame::WORLD;
  solver->robot.compact_frame_jacobian(frame_a_index, frame_type, columns, J_a);
  solver->robot.compact_frame_jacobian_time_variation(frame_a_index, frame_type, columns, dJ_a);
  solver->robot.compact_frame_jacobian(frame_b_index, frame_type, columns, J_b);
  solver->robot.compact_frame_jacobian_time_variation(frame_b_index, frame_type, columns, dJ_b);
  gather_qd();

  // Computing error
  Eigen::Affine3d T_world_a = solver->robot.get_T_world_frame(frame_a_index);
  Eigen::Affine3d T_world_b = solver->robot.get_T_world_frame(frame_b_index);
  Eigen::Matrix3d R_world_a = T_world_a.rotation();
  Eigen::Matrix3d R_a_b_real = T_world_a.rotation().transpose() * T_world_b.rotation();
  Eigen::Matrix3d M = (R_a_b * R_a_b_real.transpose()).matrix();
  Eigen::Vector3d orientation_error_world = R_world_a * pinocchio::log3(M);

  // Computing A and b
  Eigen::Vector3d world_omega_a = J_a.bottomRows(3) * qd_columns;
  Eigen::Vector3d world_omega_b = J_b.bottomRows(3) * qd_columns;
  Eigen::Vector3d world_omega_a_b_real = world_omega_b - world_omega_a;
  Eigen::Vector3d velocity_error_world = R_world_a * omega_a_b - world_omega_a_b_real;

  Eigen::Vector3d desired_acceleration = kp * orientation_error_world + get_kd() * velocity_error_world + domega_a_b;

  Eigen::Matrix3d Jlog;
  pinocchio::Jlog3(M, Jlog);

  // Acceleration is: J * qdd + dJ * qd
  Eigen::Vector3d dJ_qd = dJ_b.bottomRows(3) * qd_columns - dJ_a.bottomRows(3) * qd_columns;
  int rows = mask.rows();
  A.resize(rows, columns.size());
  b.resize(rows, 1);
  error.resize(rows, 1);
  derror.resize(rows, 1);
  mask.apply(Jlog * (J_b.bottomRows(3) - J_a.bottomRows(3)), A);
  mask.apply(desired_acceleration - Jlog * dJ_qd, b);
  mask.apply(orientation_error_world, error);
  mask.apply(velocity_error_world, derror);
}

std::string RelativeOrientationTask::type_name()
{
  return "relative_orientation";
}

std::string RelativeOrientationTask::error_unit()
{
  return "rad";
}
}  // namespace placo::dynamics