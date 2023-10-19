#include "placo/dynamics/mimic_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
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
  error = Eigen::MatrixXd(mimics.size(), 1);
  derror = Eigen::MatrixXd(mimics.size(), 1);
  A.setZero();
  b.setZero();

  int k = 0;
  for (auto& entry : mimics)
  {
    double q_target = solver->robot.state.q[entry.first + 1];
    double qd_target = solver->robot.state.qd[entry.first];
    double q_source = solver->robot.state.q[mimics[entry.first].source + 1];
    double qd_source = solver->robot.state.qd[mimics[entry.first].source];
    double ratio = mimics[entry.first].ratio;

    double desired_ddq = kp * (q_target - q_source * ratio) + get_kd() * (qd_target - qd_source * ratio);

    A(k, entry.first) = -1;
    A(k, mimics[entry.first].source) = ratio;
    b(k, 0) = desired_ddq;

    error(k, 0) = q_target - q_source;
    derror(k, 0) = qd_target - qd_source;

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
}  // namespace placo::dynamics