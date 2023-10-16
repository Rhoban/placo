#include "placo/dynamics/joints_task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo
{
namespace dynamics
{
StaticTask::StaticTask()
{
}

void StaticTask::update()
{
  A = Eigen::MatrixXd::Identity(solver->N, solver->N);
  b = Eigen::VectorXd::Zero(solver->N);
}

std::string StaticTask::type_name()
{
  return "static";
}

std::string StaticTask::error_unit()
{
  return "dof";
}
}  // namespace dynamics
}  // namespace placo