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

void RelativeOrientationTask::update()
{
  // Computing J and dJ
  Eigen::MatrixXd Ja =
      solver->robot.frame_jacobian(frame_a_index, pinocchio::ReferenceFrame::WORLD).block(3, 0, 3, solver->N);
  Eigen::MatrixXd dJa = solver->robot.frame_jacobian_time_variation(frame_a_index, pinocchio::ReferenceFrame::WORLD)
                            .block(3, 0, 3, solver->N);

  Eigen::MatrixXd Jb =
      solver->robot.frame_jacobian(frame_b_index, pinocchio::ReferenceFrame::WORLD).block(3, 0, 3, solver->N);
  Eigen::MatrixXd dJb = solver->robot.frame_jacobian_time_variation(frame_b_index, pinocchio::ReferenceFrame::WORLD)
                            .block(3, 0, 3, solver->N);

  // Computing error
  Eigen::Affine3d T_world_a = solver->robot.get_T_world_frame(frame_a_index);
  Eigen::Affine3d T_world_b = solver->robot.get_T_world_frame(frame_b_index);
  Eigen::Matrix3d R_world_a = T_world_a.rotation();
  Eigen::Matrix3d R_a_b_real = T_world_a.rotation().transpose() * T_world_b.rotation();
  Eigen::Matrix3d M = (R_a_b * R_a_b_real.transpose()).matrix();
  Eigen::Vector3d orientation_error_world = R_world_a * pinocchio::log3(M);

  // Computing A and b
  Eigen::Vector3d world_omega_a = Ja * solver->robot.state.qd;
  Eigen::Vector3d world_omega_b = Jb * solver->robot.state.qd;
  Eigen::Vector3d world_omega_a_b_real = world_omega_b - world_omega_a;
  Eigen::Vector3d velocity_error_world = R_world_a * omega_a_b - world_omega_a_b_real;

  Eigen::Vector3d desired_acceleration = kp * orientation_error_world + get_kd() * velocity_error_world + domega_a_b;

  Eigen::Matrix3d Jlog;
  pinocchio::Jlog3(M, Jlog);

  // Acceleration is: J * qdd + dJ * qd
  A = mask.apply(Jlog * (Jb - Ja));
  b = mask.apply(desired_acceleration - Jlog * (dJb * solver->robot.state.qd - dJa * solver->robot.state.qd));
  error = mask.apply(orientation_error_world);
  derror = mask.apply(velocity_error_world);
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