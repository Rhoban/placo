#include "placo/dynamics/joints_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
JointsTask::JointsTask()
{
}

void JointsTask::set_joint(std::string joint, double target, double velocity, double acceleration)
{
  joints[joint] = target;
  djoints[joint] = velocity;
  ddjoints[joint] = acceleration;
}

double JointsTask::get_joint(std::string joint)
{
  if (!joints.count(joint))
  {
    throw std::runtime_error("Joint '" + joint + "' not found in task");
  }

  return joints[joint];
}

void JointsTask::update()
{
  A = Eigen::MatrixXd(joints.size(), solver->N);
  b = Eigen::MatrixXd(joints.size(), 1);
  error = Eigen::MatrixXd(joints.size(), 1);
  derror = Eigen::MatrixXd(joints.size(), 1);
  A.setZero();

  int k = 0;
  for (auto& entry : joints)
  {
    double q = solver->robot.get_joint(entry.first);
    double dq = solver->robot.state.qd[solver->robot.get_joint_v_offset(entry.first)];
    double target_dq = djoints.count(entry.first) ? djoints[entry.first] : 0;
    double target_ddq = ddjoints.count(entry.first) ? ddjoints[entry.first] : 0;

    double desired_ddq = kp * (entry.second - q) + get_kd() * (target_dq - dq) + target_ddq;

    A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
    b(k, 0) = desired_ddq;
    error(k, 0) = entry.second - q;
    derror(k, 0) = target_dq - dq;

    k += 1;
  }
}

std::string JointsTask::type_name()
{
  return "joints";
}

std::string JointsTask::error_unit()
{
  return "dof";
}
}  // namespace placo::dynamics