#include "placo/dynamics/position_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
PositionTask::PositionTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world)
{
  this->frame_index = frame_index;
  this->target_world = target_world;
}

void PositionTask::update()
{
  pinocchio::ReferenceFrame frame_type = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  // Computing J and dJ
  Eigen::MatrixXd J = solver->robot.frame_jacobian(frame_index, frame_type).block(0, 0, 3, solver->N);
  Eigen::MatrixXd dJ = solver->robot.frame_jacobian_time_variation(frame_index, frame_type).block(0, 0, 3, solver->N);

  // Computing error
  Eigen::Affine3d T_world_frame = solver->robot.get_T_world_frame(frame_index);
  mask.R_local_world = T_world_frame.rotation().transpose();
  Eigen::Vector3d position_world = T_world_frame.translation();
  Eigen::Vector3d position_error = target_world - position_world;

  // Computing A and b
  Eigen::Vector3d velocity_world = J * solver->robot.state.qd;
  Eigen::Vector3d velocity_error = dtarget_world - velocity_world;

  Eigen::Vector3d desired_acceleration = kp * position_error + get_kd() * velocity_error + ddtarget_world;

  // Acceleration is: J * qdd + dJ * qd
  A = mask.apply(J);
  b = mask.apply(desired_acceleration - dJ * solver->robot.state.qd);
  error = mask.apply(position_error);
  derror = mask.apply(velocity_error);
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