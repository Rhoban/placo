#include "placo/dynamics/position_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
PositionTask::PositionTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world)
{
  this->frame_index = frame_index;
  this->target_world = target_world;
}

void PositionTask::support()
{
  columns = solver->robot.frame_support(frame_index);
}

void PositionTask::fill()
{
  pinocchio::ReferenceFrame frame_type = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  // Computing J and dJ
  solver->robot.compact_frame_jacobian(frame_index, frame_type, columns, J_a);
  solver->robot.compact_frame_jacobian_time_variation(frame_index, frame_type, columns, dJ_a);
  gather_qd();

  // Computing error
  const pinocchio::SE3& T_world_frame = solver->robot.data->oMf[frame_index];
  mask.R_local_world = T_world_frame.rotation().transpose();
  Eigen::Vector3d position_world = T_world_frame.translation();
  Eigen::Vector3d position_error = target_world - position_world;

  // Computing A and b
  Eigen::Vector3d velocity_world = J_a.topRows(3) * qd_columns;
  Eigen::Vector3d velocity_error = dtarget_world - velocity_world;

  Eigen::Vector3d desired_acceleration = kp * position_error + get_kd() * velocity_error + ddtarget_world;

  // Acceleration is: J * qdd + dJ * qd
  int rows = mask.rows();
  A.resize(rows, columns.size());
  b.resize(rows, 1);
  error.resize(rows, 1);
  derror.resize(rows, 1);
  mask.apply(J_a.topRows(3), A);
  mask.apply(desired_acceleration - dJ_a.topRows(3) * qd_columns, b);
  mask.apply(position_error, error);
  mask.apply(velocity_error, derror);
}

std::string PositionTask::type_name()
{
  return "position";
}

std::string PositionTask::error_unit()
{
  return "m";
}
}  // namespace placo::dynamics