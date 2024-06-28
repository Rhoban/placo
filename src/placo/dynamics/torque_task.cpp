#include "placo/dynamics/torque_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
TorqueTask::TorqueTask()
{
  tau_task = true;
}

void TorqueTask::set_torque(std::string joint, double torque, double kp, double kd)
{
  torques[joint].torque = torque;
  torques[joint].kp = kp;
  torques[joint].kd = kd;
}

void TorqueTask::reset_torque(std::string joint)
{
  torques.erase(joint);
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
    TargetTau target = entry.second;
    A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
    b(k, 0) = target.torque + target.kp * solver->robot.get_joint(entry.first) -
              target.kd * solver->robot.get_joint_velocity(entry.first);
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