#include "placo/dynamics/joints_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo
{
namespace dynamics
{
JointsTask::JointsTask()
{
}

void JointsTask::set_joint(std::string joint, double target)
{
  joints[joint] = target;
}

void JointsTask::update()
{
  A = Eigen::MatrixXd(joints.size(), solver->N);
  b = Eigen::MatrixXd(joints.size(), 1);
  A.setZero();

  int k = 0;
  for (auto& entry : joints)
  {
    double q = solver->robot.get_joint(entry.first);
    double dq = solver->robot.state.qd[solver->robot.get_joint_v_offset(entry.first)];
    double desired_ddq = kp * (entry.second - q) - 2 * sqrt(kp) * dq;

    A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
    b(k, 0) = desired_ddq;

    k += 1;
  }
}

std::string JointsTask::type_name()
{
  return "position";
}

std::string JointsTask::error_unit()
{
  return "dof";
}
}  // namespace dynamics
}  // namespace placo