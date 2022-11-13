#include "placo/control/task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
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
    A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
    b(k, 0) = entry.second - solver->robot.get_joint(entry.first);

    k += 1;
  }
}

std::string JointsTask::type_name()
{
  return "joints";
}

std::string JointsTask::error_unit()
{
  return "dof-rads";
}
}  // namespace placo