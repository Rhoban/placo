#include "placo/dynamics/relative_position_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{

RelativePositionTask::RelativePositionTask(model::RobotWrapper::FrameIndex frame_a_index,
                                           model::RobotWrapper::FrameIndex frame_b_index, Eigen::Vector3d target)
{
  this->frame_a_index = frame_a_index;
  this->frame_b_index = frame_b_index;
  this->target = target;
}

void RelativePositionTask::support()
{
  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a_index),
                                      solver->robot.frame_support(frame_b_index), columns);
}

void RelativePositionTask::fill()
{
  // Transformation matrices
  Eigen::Affine3d w_T_a = solver->robot.get_T_world_frame(frame_a_index);
  Eigen::Affine3d w_T_b = solver->robot.get_T_world_frame(frame_b_index);
  Eigen::Matrix3d a_R_w = w_T_a.rotation().transpose();

  // AB vector expressed in world and in a
  Eigen::Vector3d w_AB = w_T_b.translation() - w_T_a.translation();
  Eigen::Vector3d a_AB = a_R_w * w_AB;

  // Computing J and dJ for frame_a and frame_b
  pinocchio::ReferenceFrame frame_type = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  solver->robot.compact_frame_jacobian(frame_a_index, frame_type, columns, J_a);
  solver->robot.compact_frame_jacobian_time_variation(frame_a_index, frame_type, columns, dJ_a);
  solver->robot.compact_frame_jacobian(frame_b_index, frame_type, columns, J_b);
  solver->robot.compact_frame_jacobian_time_variation(frame_b_index, frame_type, columns, dJ_b);
  gather_qd();

  // Rotation velocity of frame a in the world frame
  Eigen::Vector3d w_omega_a = J_a.bottomRows(3) * qd_columns;
  Eigen::Vector3d a_omega_w = -a_R_w * w_omega_a;

  // Velocity of the error expressed in a
  Eigen::Vector3d w_dAB = (J_b.topRows(3) - J_a.topRows(3)) * qd_columns;
  Eigen::Vector3d a_dAB = a_omega_w.cross(a_AB) + a_R_w * w_dAB;

  // Computing error
  Eigen::Vector3d position_error = target - a_AB;
  Eigen::Vector3d velocity_error = dtarget - a_dAB;
  Eigen::Vector3d desired_acceleration = kp * position_error + get_kd() * velocity_error + ddtarget;

  // The acceleration of AB in a is expressed as: J * ddq + e
  Eigen::Matrix<double, 3, Eigen::Dynamic> J = pinocchio::skew(a_AB) * a_R_w * J_a.bottomRows(3);
  J += a_R_w * (J_b.topRows(3) - J_a.topRows(3));

  Eigen::Vector3d e = 2 * pinocchio::skew(a_omega_w) * a_R_w * w_dAB;
  e += 2 * pinocchio::skew(a_omega_w) * pinocchio::skew(a_omega_w) * a_AB;
  e += a_R_w * (dJ_b.bottomRows(3) - dJ_a.bottomRows(3)) * qd_columns;
  e += pinocchio::skew(a_AB) * a_R_w * dJ_a.bottomRows(3) * qd_columns;

  int rows = mask.rows();
  A.resize(rows, columns.size());
  b.resize(rows, 1);
  error.resize(rows, 1);
  derror.resize(rows, 1);
  mask.apply(J, A);
  mask.apply(-e + desired_acceleration, b);
  mask.apply(position_error, error);
  mask.apply(velocity_error, derror);
}

std::string RelativePositionTask::type_name()
{
  return "relative_position";
}

std::string RelativePositionTask::error_unit()
{
  return "m";
}
}  // namespace placo::dynamics