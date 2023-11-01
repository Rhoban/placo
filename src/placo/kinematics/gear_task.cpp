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
  int source_id = solver->robot.get_joint_v_offset(source);

  gears[target_id] = { target_id, source_id, ratio };
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
    auto& gear = entry.second;

    double q_target = solver->robot.state.q[gear.target + 1];
    double q_source = solver->robot.state.q[gear.source + 1];

    A(k, gear.target) = -1;
    A(k, gear.source) = gear.ratio;
    b(k, 0) = q_target - q_source * gear.ratio;

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