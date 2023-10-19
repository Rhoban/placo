#include "placo/kinematics/task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
MimicTask::MimicTask()
{
}

void MimicTask::set_mimic(std::string target, std::string source, double ratio)
{
  mimics[solver->robot.get_joint_v_offset(target)] = { solver->robot.get_joint_v_offset(source), ratio };
}

void MimicTask::update()
{
  A = Eigen::MatrixXd(mimics.size(), solver->N);
  b = Eigen::MatrixXd(mimics.size(), 1);
  A.setZero();
  b.setZero();

  int k = 0;
  for (auto& entry : mimics)
  {
    A(k, entry.first) = 1;
    A(k, entry.second.source) = -entry.second.ratio;
    k += 1;
  }
}

std::string MimicTask::type_name()
{
  return "mimic";
}

std::string MimicTask::error_unit()
{
  return "dof";
}
}  // namespace placo::kinematics