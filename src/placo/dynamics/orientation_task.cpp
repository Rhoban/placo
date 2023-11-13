#include "placo/dynamics/position_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
OrientationTask::OrientationTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
{
  this->frame_index = frame_index;
  this->R_world_frame = R_world_frame;
}

void OrientationTask::update()
{
  Eigen::Affine3d T_world_frame = solver->robot.get_T_world_frame(frame_index);

  pinocchio::ReferenceFrame frame_type = pinocchio::ReferenceFrame::WORLD;

  // Computing J and dJ
  Eigen::MatrixXd J = solver->robot.frame_jacobian(frame_index, frame_type).block(3, 0, 3, solver->N);
  Eigen::MatrixXd dJ = solver->robot.frame_jacobian_time_variation(frame_index, frame_type).block(3, 0, 3, solver->N);

  // Computing error
  Eigen::Matrix3d M;
  Eigen::Vector3d orientation_error;
  M = (R_world_frame * T_world_frame.linear().transpose()).matrix();
  orientation_error = pinocchio::log3(M);

  // Computing A and b
  Eigen::Vector3d velocity_world = J * solver->robot.state.qd;
  Eigen::Vector3d velocity_error = omega_world - velocity_world;

  Eigen::Vector3d desired_acceleration = kp * orientation_error + get_kd() * velocity_error + domega_world;

  mask.R_local_world = R_world_frame.transpose();
  A = mask.apply(J);
  b = mask.apply(desired_acceleration - dJ * solver->robot.state.qd);
  error = mask.apply(orientation_error);
  derror = mask.apply(velocity_error);
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