#include "placo/dynamics/position_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo
{
namespace dynamics
{

PositionTask::PositionTask(RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world)
{
  this->frame_index = frame_index;
  this->target_world = target_world;
}

void PositionTask::update()
{
  // Computing J and dJ
  Eigen::MatrixXd J = solver->robot.frame_jacobian(frame_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
                          .block(0, 0, 3, solver->N);
  Eigen::MatrixXd dJ =
      solver->robot.frame_jacobian_time_variation(frame_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
          .block(0, 0, 3, solver->N);

  // Computing error
  Eigen::Vector3d position_world = solver->robot.get_T_world_frame(frame_index).translation();
  Eigen::Vector3d position_error = target_world - position_world;

  // Computing A and b
  Eigen::Vector3d velocity_world = J * solver->robot.state.qd;
  Eigen::Vector3d velocity_error = dtarget_world - velocity_world;

  Eigen::Vector3d desired_acceleration = kp * position_error + get_kd() * velocity_error;

  // Acceleration is: J * qdd + dJ * qd
  A = J(mask.indices, Eigen::placeholders::all);
  b = (desired_acceleration - dJ * solver->robot.state.qd)(mask.indices, Eigen::placeholders::all);
}

std::string PositionTask::type_name()
{
  return "position";
}

std::string PositionTask::error_unit()
{
  return "m";
}
}  // namespace dynamics
}  // namespace placo