#include "placo/dynamics/torque_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
TorqueTask::TorqueTask()
{
  tau_task = true;
}

void TorqueTask::set_torque(std::string joint, double torque)
{
  torques[joint] = torque;
}

void TorqueTask::update()
{
  A = Eigen::MatrixXd(torques.size(), solver->N);
  b = Eigen::MatrixXd(torques.size(), 1);
  error = Eigen::MatrixXd(torques.size(), 1);
  derror = Eigen::MatrixXd(torques.size(), 1);
  error.setZero();
  derror.setZero();
  A.setZero();
  b.setZero();

  int k = 0;
  for (auto& entry : torques)
  {
    A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
    b(k, 0) = entry.second;
    k++;
  }
}

std::string TorqueTask::type_name()
{
  return "torques";
}

std::string TorqueTask::error_unit()
{
  return "-";
}
}  // namespace placo::dynamics