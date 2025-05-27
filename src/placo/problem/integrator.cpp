#include <algorithm>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include "placo/problem/integrator.h"
#include "placo/problem/problem.h"

namespace placo::problem
{
double Integrator::Trajectory::value(double t, int diff)
{
  t -= t_start;

  Integrator::check_diff(order, diff);

  int k = std::floor(t / dt);

  if (k < 0)
    k = 0;

  if (k >= variable_value.size())
    k = variable_value.size() - 1;

  double remaining_dt = fmax(0, fmin(dt, t - k * dt));

  if (diff == order)
  {
    return variable_value[k];
  }
  else
  {
    auto AB = AB_matrices(M, order, remaining_dt);
    Eigen::MatrixXd Ar = AB.first;
    Eigen::MatrixXd Br = AB.second;

    Eigen::VectorXd result = Ar * keyframes[k] + Br * variable_value[k];

    return result[diff];
  }
}

double Integrator::Trajectory::duration()
{
  return keyframes.size() * dt;
}

Integrator::Integrator()
{
}

Integrator::Integrator(Variable& variable_, Expression X0, Eigen::MatrixXd system_matrix, double dt)
  : variable(&variable_), X0(X0), dt(dt), M(system_matrix)
{
  order = system_matrix.rows() - 1;

  N = variable->size();

  auto AB = AB_matrices(M, order, dt);
  A = AB.first;
  B = AB.second;

  // Computing final transition matrix and powers of A
  final_transition_matrix = Eigen::MatrixXd(order, N);
  final_transition_matrix.setZero();

  Eigen::MatrixXd Ak(order, order);
  Ak.setIdentity();
  a_powers[0] = Ak;

  for (int step = 0; step < N; step++)
  {
    final_transition_matrix.block(0, N - step - 1, order, 1) = Ak * B;
    Ak = A * Ak;
    a_powers[step + 1] = Ak;
  }
}

Integrator::Integrator(Variable& variable_, Expression X0, int order, double dt)
  : Integrator(variable_, X0, upper_shift_matrix(order), dt)
{
  if (X0.rows() != order)
  {
    throw std::runtime_error("Integrator: X0 should have " + std::to_string(order) + " rows (same as order)");
  }
}

Integrator::~Integrator()
{
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Integrator::AB_matrices(Eigen::MatrixXd& M, int order, double dt)
{
  // Computing A and B transition matrices
  Eigen::MatrixXd Me = (M * dt).exp();
  Eigen::MatrixXd A = Me.block(0, 0, order, order);
  Eigen::MatrixXd B = Me.block(0, order, order, 1);

  return std::pair<Eigen::MatrixXd, Eigen::VectorXd>(A, B);
}

Eigen::MatrixXd Integrator::upper_shift_matrix(int order)
{
  Eigen::MatrixXd M(order + 1, order + 1);
  M.setZero();

  for (int k = 0; k < order; k++)
  {
    M(k, k + 1) = 1.0;
  }

  return M;
}

void Integrator::check_diff(int order, int diff, bool allow_all)
{
  int diff_min = allow_all ? -1 : 0;

  if (diff < diff_min || diff > order)
  {
    std::ostringstream oss;
    oss << "Asked differentiation order of " << diff << " for an integrator of order " << order;
    throw std::runtime_error(oss.str());
  }
}

Expression Integrator::expr(int step, int diff)
{
  check_diff(order, diff, true);

  if (step == -1)
  {
    step = variable->size();
  }

  if (step < 0 || step > variable->size())
  {
    std::ostringstream oss;
    oss << "Asking an expression for step " << step << ", should be between " << 0 << " and " << variable->size();
    throw std::runtime_error(oss.str());
  }

  if (diff == order)
  {
    // At order, we just select the relevant variable
    return variable->expr(step, 1);
  }
  else
  {
    Expression e;
    int rows = (diff == -1) ? order : 1;
    e.A = Eigen::MatrixXd(rows, variable->k_end);
    e.A.setZero();
    e.b = Eigen::VectorXd(rows);
    e.b.setZero();

    if (diff == -1)
    {
      e.A.block(0, variable->k_start, rows, step) = final_transition_matrix.block(0, N - step, rows, step);
      e = e + a_powers[step] * X0;
    }
    else
    {
      e.A.block(0, variable->k_start, 1, step) = final_transition_matrix.block(diff, N - step, 1, step);
      e = e + (a_powers[step] * X0).slice(diff, 1);
    }

    return e;
  }
}

Expression Integrator::expr_t(double t, int diff)
{
  t -= t_start;

  int step = std::max<int>(std::min<int>(variable->size() - 1, t / dt), 0);

  if (diff == order)
  {
    return variable->expr(step, 1);
  }
  else
  {
    double remaining_dt = t - step * dt;

    auto AB = AB_matrices(M, order, remaining_dt);
    Eigen::MatrixXd Ar = AB.first;
    Eigen::MatrixXd Br = AB.second;

    Expression e = (Ar * expr(step)) + Br * variable->expr(step, 1);

    if (diff > -1)
    {
      e.A = Eigen::MatrixXd(e.A.block(diff, 0, 1, e.A.cols()));
      e.b = Eigen::MatrixXd(e.b.block(diff, 0, 1, e.b.cols()));
    }

    return e;
  }
}

double Integrator::value(double t, int diff)
{
  update_trajectory();

  return trajectory.value(t, diff);
}

Integrator::Trajectory Integrator::get_trajectory()
{
  update_trajectory();

  return trajectory;
}

void Integrator::update_trajectory()
{
  if (variable->version == 0)
  {
    throw std::runtime_error("Trying to get the trajectory with a variable that was not solved");
  }

  if (version != variable->version)
  {
    // Updating trajectory data
    trajectory.M = M;
    trajectory.dt = dt;
    trajectory.order = order;
    trajectory.t_start = t_start;
    trajectory.variable_value = variable->value;

    // Updating keyframes
    Eigen::VectorXd X = X0.value(variable->problem->x);
    trajectory.keyframes[0] = X;

    for (int k = 1; k <= variable->size(); k++)
    {
      X = A * X + B * variable->value[k - 1];
      trajectory.keyframes[k] = X;
    }

    version = variable->version;
  }
}

}  // namespace placo::problem