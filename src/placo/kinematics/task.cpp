#include "placo/kinematics/task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void Task::support()
{
  columns.clear();
}

void Task::fill()
{
  throw std::logic_error("Task: " + type_name() + " should implement fill() or update()");
}

void Task::update()
{
  support();
  fill();
}

Eigen::MatrixXd Task::dense_A() const
{
  if (columns.empty() && A.cols() == solver->N)
  {
    return A;
  }

  // Compact matrix (possibly without any column, for instance for a frame attached to the world)
  Eigen::MatrixXd full = Eigen::MatrixXd::Zero(A.rows(), solver->N);
  for (int k = 0; k < (int)columns.size(); k++)
  {
    full.col(columns[k]) = A.col(k);
  }
  return full;
}

Eigen::MatrixXd Task::error()
{
  return b;
}

double Task::error_norm()
{
  return b.norm();
}
}  // namespace placo::kinematics
