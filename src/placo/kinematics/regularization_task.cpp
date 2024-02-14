#include "placo/kinematics/regularization_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void RegularizationTask::update()
{
  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  // Floating base is not regularized by this task
  Eigen::MatrixXd I = Eigen::MatrixXd(solver->N, solver->N);
  I.setIdentity();

  A = Eigen::MatrixXd(solver->N - 6, solver->N);
  A.block(0, 0, solver->N - 6, solver->N) = I.block(6, 0, solver->N - 6, solver->N);

  b = Eigen::MatrixXd(solver->N - 6, 1);
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
}  // namespace placo::kinematics