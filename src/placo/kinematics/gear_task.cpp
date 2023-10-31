#include "placo/kinematics/task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
GearTask::GearTask()
{
}

void GearTask::set_gear(std::string target, std::string source, double ratio)
{
  gears[solver->robot.get_joint_v_offset(target)] = { solver->robot.get_joint_v_offset(source), ratio };
}

void GearTask::update()
{
  A = Eigen::MatrixXd(gears.size(), solver->N);
  b = Eigen::MatrixXd(gears.size(), 1);
  A.setZero();
  b.setZero();

  int k = 0;
  for (auto& entry : gears)
  {
    A(k, entry.first) = 1;
    A(k, entry.second.source) = -entry.second.ratio;
    k += 1;
  }
}

std::string GearTask::type_name()
{
  return "gear";
}

std::string GearTask::error_unit()
{
  return "dof";
}
}  // namespace placo::kinematics