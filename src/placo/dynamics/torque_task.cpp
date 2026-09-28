#include <algorithm>
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

void TorqueTask::support()
{
  // This task is on the torques (the columns are the rows of tau it selects)
  columns.clear();
  for (auto& entry : torques)
  {
    columns.push_back(solver->robot.get_joint_v_offset(entry.first));
  }
  std::sort(columns.begin(), columns.end());
  columns.erase(std::unique(columns.begin(), columns.end()), columns.end());
}

void TorqueTask::fill()
{
  A.setZero(torques.size(), columns.size());
  b.setZero(torques.size(), 1);
  error.setZero(torques.size(), 1);
  derror.setZero(torques.size(), 1);

  int k = 0;
  for (auto& entry : torques)
  {
    TargetTau target = entry.second;
    int offset = solver->robot.get_joint_v_offset(entry.first);
    A(k, std::lower_bound(columns.begin(), columns.end(), offset) - columns.begin()) = 1;
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