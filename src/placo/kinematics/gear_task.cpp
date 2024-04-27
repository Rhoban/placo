#include "placo/kinematics/task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
GearTask::GearTask()
{
}

void GearTask::set_gear(std::string target, std::string source, double ratio)
{
  int target_id = solver->robot.get_joint_v_offset(target);
  gears[target_id].clear();

  add_gear(target, source, ratio);
}

void GearTask::add_gear(std::string target, std::string source, double ratio)
{
  int target_id = solver->robot.get_joint_v_offset(target);
  int source_id = solver->robot.get_joint_v_offset(source);

  gears[target_id][source_id] = ratio;
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
    int target = entry.first;
    A(k, target) = -1;
    double q_target = solver->robot.state.q[target + 1];
    b(k, 0) = q_target;

    for (auto& gear_source : entry.second)
    {
      int source = gear_source.first;
      double ratio = gear_source.second;
      double q_source = solver->robot.state.q[source + 1];

      A(k, source) = ratio;
      b(k, 0) -= q_source * ratio;
    }

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