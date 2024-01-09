#include "placo/dynamics/gear_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
GearTask::GearTask()
{
}

void GearTask::set_gear(std::string target, std::string source, double ratio)
{
  gears.clear();
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
  error = Eigen::MatrixXd(gears.size(), 1);
  derror = Eigen::MatrixXd(gears.size(), 1);
  A.setZero();
  b.setZero();

  int k = 0;
  for (auto& entry : gears)
  {
    int target = entry.first;
    double desired_q = 0;
    double desired_qd = 0;

    double q_target = solver->robot.state.q[target + 1];
    double qd_target = solver->robot.state.qd[target];
    A(k, target) = -1;

    for (auto& gear : entry.second)
    {
      int source = gear.first;
      double ratio = gear.second;

      desired_q += solver->robot.state.q[source + 1] * ratio;
      desired_qd += solver->robot.state.qd[source] * ratio;
      A(k, source) = ratio;
    }

    double desired_ddq = kp * (q_target - desired_q) + get_kd() * (qd_target - desired_qd);

    b(k, 0) = desired_ddq;

    error(k, 0) = q_target - desired_q;
    derror(k, 0) = qd_target - desired_qd;

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