#include <algorithm>
#include "placo/kinematics/task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
JointsTask::JointsTask()
{
}

void JointsTask::set_joint(std::string joint, double target)
{
  joints[joint] = target;
}

double JointsTask::get_joint(std::string joint)
{
  if (!joints.count(joint))
  {
    throw std::runtime_error("Joint '" + joint + "' not found in task");
  }

  return joints[joint];
}

void JointsTask::support()
{
  columns.clear();
  for (auto& entry : joints)
  {
    columns.push_back(solver->robot.get_joint_v_offset(entry.first));
  }
  std::sort(columns.begin(), columns.end());
  columns.erase(std::unique(columns.begin(), columns.end()), columns.end());
}

void JointsTask::fill()
{
  A.setZero(joints.size(), columns.size());
  b.resize(joints.size(), 1);

  int k = 0;
  for (auto& entry : joints)
  {
    int offset = solver->robot.get_joint_v_offset(entry.first);
    A(k, std::lower_bound(columns.begin(), columns.end(), offset) - columns.begin()) = 1;
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
  return "dof";
}
}  // namespace placo::kinematics