#include "placo/dynamics/com_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo
{
namespace dynamics
{

CoMTask::CoMTask(Eigen::Vector3d target_world)
{
  this->target_world = target_world;
}

void CoMTask::update()
{
  // Computing J and dJ
  Eigen::MatrixXd J = solver->robot.com_jacobian();
  Eigen::MatrixXd dJ = solver->robot.com_jacobian_time_variation();

  // Computing error
  Eigen::Vector3d position_world = solver->robot.com_world();
  Eigen::Vector3d position_error = target_world - position_world;

  // Computing A and b
  Eigen::Vector3d velocity_world = J * solver->robot.state.qd;
  Eigen::Vector3d velocity_error = dtarget_world - velocity_world;

  Eigen::Vector3d desired_acceleration = kp * position_error + get_kd() * velocity_error;

  // Acceleration is: J * qdd + dJ * qd
  A = J(mask.indices, Eigen::placeholders::all);
  b = (desired_acceleration - dJ * solver->robot.state.qd)(mask.indices, Eigen::placeholders::all);
}

std::string CoMTask::type_name()
{
  return "com";
}

std::string CoMTask::error_unit()
{
  return "m";
}
}  // namespace dynamics
}  // namespace placo