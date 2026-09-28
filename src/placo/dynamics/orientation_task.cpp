#include "placo/dynamics/position_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
OrientationTask::OrientationTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
{
  this->frame_index = frame_index;
  this->R_world_frame = R_world_frame;
}

void OrientationTask::support()
{
  columns = solver->robot.frame_support(frame_index);
}

void OrientationTask::fill()
{
  pinocchio::SE3 T_world_frame = solver->robot.data->oMf[frame_index];

  pinocchio::ReferenceFrame frame_type = pinocchio::ReferenceFrame::WORLD;

  // Computing J and dJ
  solver->robot.compact_frame_jacobian(frame_index, frame_type, columns, J_a);
  solver->robot.compact_frame_jacobian_time_variation(frame_index, frame_type, columns, dJ_a);
  gather_qd();

  // Computing error
  Eigen::Matrix3d M = R_world_frame * T_world_frame.rotation().transpose();
  Eigen::Vector3d orientation_error = pinocchio::log3(M);

  // Computing A and b
  Eigen::Vector3d velocity_world = J_a.bottomRows(3) * qd_columns;
  Eigen::Vector3d velocity_error = omega_world - velocity_world;

  Eigen::Vector3d desired_acceleration = kp * orientation_error + get_kd() * velocity_error + domega_world;

  mask.R_local_world = R_world_frame.transpose();
  int rows = mask.rows();
  A.resize(rows, columns.size());
  b.resize(rows, 1);
  error.resize(rows, 1);
  derror.resize(rows, 1);
  mask.apply(J_a.bottomRows(3), A);
  mask.apply(desired_acceleration - dJ_a.bottomRows(3) * qd_columns, b);
  mask.apply(orientation_error, error);
  mask.apply(velocity_error, derror);
}

std::string OrientationTask::type_name()
{
  return "orientation";
}

std::string OrientationTask::error_unit()
{
  return "rad";
}
}  // namespace placo::dynamics