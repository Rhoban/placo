#include "placo/dynamics/gear_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
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
  error = Eigen::MatrixXd(gears.size(), 1);
  derror = Eigen::MatrixXd(gears.size(), 1);
  A.setZero();
  b.setZero();

  int k = 0;
  for (auto& entry : gears)
  {
    auto& gear = entry.second;
    double q_target = solver->robot.state.q[gear.target + 1];
    double qd_target = solver->robot.state.qd[gear.target];
    double q_source = solver->robot.state.q[gear.source + 1];
    double qd_source = solver->robot.state.qd[gear.source];
    double ratio = gear.ratio;

    double desired_ddq = kp * (q_target - q_source * ratio) + get_kd() * (qd_target - qd_source * ratio);

    A(k, gear.target) = -1;
    A(k, gear.source) = ratio;
    b(k, 0) = desired_ddq;

    error(k, 0) = q_target - q_source;
    derror(k, 0) = qd_target - qd_source;

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
}  // namespace placo::dynamics