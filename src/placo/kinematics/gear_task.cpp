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
    double q_target = solver->robot.state.q[entry.first + 1];
    double q_source = solver->robot.state.q[gears[entry.first].source + 1];

    A(k, entry.first) = -1;
    A(k, entry.second.source) = entry.second.ratio;
    b(k, 0) = q_target - q_source * entry.second.ratio;

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