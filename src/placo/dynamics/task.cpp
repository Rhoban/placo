#include "placo/dynamics/task.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
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

void Task::gather_qd()
{
  qd_columns.resize(columns.size());
  for (int k = 0; k < (int)columns.size(); k++)
  {
    qd_columns[k] = solver->robot.state.qd[columns[k]];
  }
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

double Task::get_kd()
{
  if (kd < 0.0)
  {
    return 2. * sqrt(kp);
  }
  else
  {
    return kd;
  }
}
}  // namespace placo::dynamics