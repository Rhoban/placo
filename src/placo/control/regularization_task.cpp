#include "placo/control/regularization_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
void RegularizationTask::update()
{
  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  A = Eigen::MatrixXd(solver->N, solver->N);
  A.setIdentity();

  b = Eigen::MatrixXd(solver->N, 1);
  b.setZero();
}

std::string RegularizationTask::type_name()
{
  return "regularization";
}

std::string RegularizationTask::error_unit()
{
  return "none";
}
}  // namespace placo