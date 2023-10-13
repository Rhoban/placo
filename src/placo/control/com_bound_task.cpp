#include "placo/control/com_bound_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
CoMBoundTask::CoMBoundTask(double bound, int dir) : bound(bound), dir(dir)
{
  equality_task = false;
  b = Eigen::MatrixXd(1, 1);
}

void CoMBoundTask::update()
{
  A = dir * solver->robot->com_jacobian()(2, Eigen::placeholders::all);
  b(0, 0) = dir * (bound - solver->robot->com_world()[2]);
}

std::string CoMBoundTask::type_name()
{
  return "com_lb";
}

std::string CoMBoundTask::error_unit()
{
  return "m";
}
}  // namespace placo