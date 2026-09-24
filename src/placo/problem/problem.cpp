#include <map>
#include <chrono>
#include "placo/problem/problem.h"
#include "placo/problem/qp_error.h"
#include "eiquadprog/eiquadprog.hpp"

namespace placo::problem
{
Problem::Problem()
{
}

Problem::~Problem()
{
  for (auto constraint : constraints)
  {
    delete constraint;
  }

  for (auto variable : variables)
  {
    delete variable;
  }

  constraints.clear();
  variables.clear();
}

Variable& Problem::add_variable(int size)
{
  Variable* variable = new Variable;
  variable->problem = this;
  variable->k_start = n_variables;
  variable->k_end = n_variables + size;
  n_variables += size;

  variables.push_back(variable);

  return *variable;
}

ProblemConstraint& Problem::add_limit(Expression expression, Eigen::VectorXd target)
{
  // -target <= expression <= target
  Eigen::VectorXd targets(target.rows() * 2);
  problem::Expression e;
  e.A.resize(expression.A.rows() * 2, expression.A.cols());
  e.b.resize(expression.b.rows() * 2, expression.b.cols());

  // Ax + b <= target
  e.A.block(0, 0, expression.A.rows(), expression.A.cols()) = expression.A;
  e.b.block(0, 0, expression.b.rows(), expression.b.cols()) = expression.b;

  // Ax + b >= -taget  =>   -Ax -b <= target
  e.A.block(expression.A.rows(), 0, expression.A.rows(), expression.A.cols()) = -expression.A;
  e.b.block(expression.b.rows(), 0, expression.b.rows(), expression.b.cols()) = -expression.b;

  targets.block(0, 0, target.rows(), 1) = target;
  targets.block(target.rows(), 0, target.rows(), 1) = target;

  return add_constraint(e <= targets);
}

ProblemConstraint& Problem::add_constraint(const ProblemConstraint& constraint_)
{
  ProblemConstraint* constraint = new ProblemConstraint;
  *constraint = constraint_;
  constraints.push_back(constraint);

  return *constraint;
}

void Problem::clear_constraints()
{
  for (auto constraint : constraints)
  {
    delete constraint;
  }

  constraints.clear();
}

void Problem::clear_variables()
{
  for (auto variable : variables)
  {
    delete variable;
  }

  variables.clear();
  n_variables = 0;
}

void Problem::get_constraint_expressions(ProblemConstraint* constraint, Eigen::MatrixXd& A, Eigen::MatrixXd& b)
{
  if (determined_variables)
  {
    Eigen::MatrixXd full_A(constraint->expression.A.rows(), n_variables);
    full_A.setZero();
    full_A.block(0, 0, constraint->expression.A.rows(), constraint->expression.A.cols()) = constraint->expression.A;
    QR.matrixQ().applyThisOnTheRight(full_A);

    A = full_A.rightCols(free_variables);
    b = constraint->expression.b + full_A.leftCols(determined_variables) * y;
  }
  else
  {
    A = constraint->expression.A;
    b = constraint->expression.b;
  }
}

void Problem::solve()
{
  n_equalities = 0;
  n_inequalities = 0;
  slack_variables = 0;

  for (auto constraint : constraints)
  {
    if (constraint->type == ProblemConstraint::Inequality)
    {
      constraint->is_active = false;
      if (constraint->priority == ProblemConstraint::Soft)
      {
        slack_variables += constraint->expression.rows();
      }
    }
    else
    {
      if (constraint->priority == ProblemConstraint::Hard)
      {
        n_equalities += constraint->expression.rows();
      }
      constraint->is_active = true;
    }
  }

  // Equality constraints
  Eigen::MatrixXd A(n_equalities, n_variables);
  Eigen::VectorXd b(n_equalities);
  A.setZero();
  b.setZero();
  int k_equality = 0;

  for (auto constraint : constraints)
  {
    if (constraint->type == ProblemConstraint::Equality && constraint->priority == ProblemConstraint::Hard)
    {
      // Ax + b = 0
      A.block(k_equality, 0, constraint->expression.rows(), constraint->expression.cols()) = constraint->expression.A;
      b.block(k_equality, 0, constraint->expression.rows(), 1) = constraint->expression.b;
      k_equality += constraint->expression.rows();
    }
  }

  free_variables = n_variables;
  determined_variables = 0;

  if (rewrite_equalities && A.rows() > 0)
  {
    // Computing QR decomposition of A.T
    QR = A.transpose().colPivHouseholderQr();

    determined_variables = QR.rank();

    if (determined_variables != A.rows())
    {
      throw QPError("QR decomposition failed to find a full rank matrix for equality constraints");
    }

    Eigen::MatrixXd R = QR.matrixR().transpose().block(0, 0, determined_variables, determined_variables);
    Eigen::MatrixXd b2 = b.transpose();
    QR.colsPermutation().applyThisOnTheRight(b2);
    b2.transposeInPlace();

    y = R.triangularView<Eigen::Lower>().solve(-b2);

    free_variables = n_variables - determined_variables;

    // Removing equality constraints
    n_equalities = 0.;
    A.resize(0, 0);
    b.resize(0);
  }

  Eigen::MatrixXd P(free_variables + slack_variables, free_variables + slack_variables);
  Eigen::VectorXd q(free_variables + slack_variables);

  P.setZero();
  q.setZero();

  // Adding regularization
  P.block(0, 0, free_variables, free_variables).setIdentity();
  P.block(0, 0, free_variables, free_variables) *= regularization;

  // Scanning the constraints (counting inequalities and equalities, building objectif function)
  for (auto constraint : constraints)
  {
    if (constraint->expression.cols() > n_variables)
    {
      throw QPError("Problem: Inconsistent problem size");
    }
    if (constraint->expression.A.rows() == 0 || constraint->expression.b.rows() == 0)
    {
      throw QPError("Problem: A or b is empty");
    }
    if (constraint->expression.A.rows() != constraint->expression.b.rows())
    {
      throw QPError("Problem: A.rows() != b.rows()");
    }

    if (constraint->type == ProblemConstraint::Inequality)
    {
      // If the constraint is hard, this will be the true inequality, else, this will be the inequality
      // enforcing the slack variable to be >= 0
      n_inequalities += constraint->expression.rows();
    }
    else if (constraint->priority == ProblemConstraint::Soft)
    {
      Eigen::MatrixXd expression_A;
      Eigen::MatrixXd expression_b;
      get_constraint_expressions(constraint, expression_A, expression_b);

      // Adding the soft constraint to the objective function
      if (use_sparsity)
      {
        Sparsity sparsity = Sparsity::detect_columns_sparsity(expression_A);

        int constraints = expression_A.rows();

        for (auto interval : sparsity.intervals)
        {
          int size = 1 + interval.end - interval.start;

          Eigen::MatrixXd block = expression_A.block(0, interval.start, constraints, size);

          P.block(interval.start, interval.start, size, size).noalias() +=
              constraint->weight * block.transpose() * block;
        }

        q.block(0, 0, expression_A.cols(), 1).noalias() +=
            constraint->weight * (expression_A.transpose() * expression_b);
      }
      else
      {
        int n = expression_A.cols();
        P.block(0, 0, n, n).noalias() += constraint->weight * (expression_A.transpose() * expression_A);
        q.block(0, 0, n, 1).noalias() += constraint->weight * (expression_A.transpose() * expression_b);
      }
    }
  }

  // Inequality constraints
  Eigen::MatrixXd G(n_inequalities, free_variables + slack_variables);
  Eigen::VectorXd h(n_inequalities);
  G.setZero();
  h.setZero();

  // Used to keep track of the hard/soft inequalities constraints
  // The hard mapping maps index from inequality row to constraint, and the soft
  // mapping maps index from slack variables to the constraint.
  std::map<int, ProblemConstraint*> hard_inequalities_mapping;
  std::map<int, ProblemConstraint*> soft_inequalities_mapping;

  int k_inequality = 0;
  int k_slack = 0;

  // Slack variables should be positive
  for (int slack = 0; slack < slack_variables; slack += 1)
  {
    // s_i >= 0
    G(k_inequality, free_variables + slack) = 1;
    k_inequality += 1;
  }

  for (auto constraint : constraints)
  {
    if (constraint->type == ProblemConstraint::Inequality)
    {
      Eigen::MatrixXd expression_A;
      Eigen::MatrixXd expression_b;
      get_constraint_expressions(constraint, expression_A, expression_b);

      if (constraint->priority == ProblemConstraint::Hard)
      {
        // Ax + b >= 0
        G.block(k_inequality, 0, expression_A.rows(), expression_A.cols()) = expression_A;
        h.block(k_inequality, 0, expression_b.rows(), 1) = expression_b;

        for (int k = k_inequality; k < k_inequality + expression_A.rows(); k++)
        {
          hard_inequalities_mapping[k] = constraint;
        }
        k_inequality += expression_A.rows();
      }
      else
      {
        // min(Ax + b - s)
        // A slack variable is assigend with all "soft" inequality and a minimization is added to the problem
        Eigen::MatrixXd As(expression_A.rows(), free_variables + slack_variables);
        As.setZero();
        As.block(0, 0, expression_A.rows(), expression_A.cols()) = expression_A;

        for (int k = 0; k < expression_A.rows(); k++)
        {
          soft_inequalities_mapping[k_slack] = constraint;
          As(k, free_variables + k_slack) = -1;
          k_slack += 1;
        }

        P.noalias() += constraint->weight * (As.transpose() * As);
        q.noalias() += constraint->weight * (As.transpose() * expression_b);
      }
    }
  }

  Eigen::VectorXi active_set;
  size_t active_set_size;

  Eigen::VectorXd qp_x(free_variables + slack_variables);
  qp_x.setZero();
  double result =
      eiquadprog::solvers::solve_quadprog(P, q, A.transpose(), b, G.transpose(), h, qp_x, active_set, active_set_size);

  if (determined_variables)
  {
    Eigen::VectorXd u(n_variables, 1);
    u.setZero();
    u.topRows(determined_variables) = y;
    u.bottomRows(free_variables) = qp_x.topRows(free_variables);
    QR.matrixQ().applyThisOnTheLeft(u);
    x = u;
  }
  else
  {
    x = qp_x;
  }

  // Checking that the problem is indeed feasible
  if (result == std::numeric_limits<double>::infinity())
  {
    throw QPError("Problem: Infeasible QP (check your hard inequality constraints)");
  }

  // Checking that equality constraints were enforced, since this is not covered by above result
  if (A.rows() > 0)
  {
    Eigen::VectorXd equality_constraints = A * x.topRows(A.cols()) + b;
    for (int k = 0; k < A.rows(); k++)
    {
      if (fabs(equality_constraints[k]) > 1e-6)
      {
        throw QPError("Problem: Infeasible QP (equality constraints were not enforced)");
      }
    }
  }

  // Checking for NaNs in solution
  if (x.hasNaN())
  {
    throw QPError("Problem: NaN in the QP solution");
  }

  // Reporting on the active constraints
  for (int k = 0; k < active_set_size; k++)
  {
    int active_constraint = active_set[k];

    if (active_constraint >= 0 && hard_inequalities_mapping.count(active_constraint))
    {
      hard_inequalities_mapping[active_constraint]->is_active = true;
    }
  }

  slacks = qp_x.block(free_variables, 0, slack_variables, 1);
  for (int k = 0; k < slacks.rows(); k++)
  {
    if (slacks[k] <= 1e-6 && soft_inequalities_mapping.count(k))
    {
      soft_inequalities_mapping[k]->is_active = true;
    }
  }

  for (auto variable : variables)
  {
    variable->version += 1;
    variable->value = Eigen::VectorXd(variable->size());
    variable->value = x.block(variable->k_start, 0, variable->size(), 1);
  }
}

void Problem::dump_status()
{
  std::cout << "Problem status:" << std::endl;
  std::cout << "  - Variables: " << n_variables << std::endl;
  std::cout << "  - Inequalities: " << n_inequalities << std::endl;
  std::cout << "  - Equalities: " << n_equalities << std::endl;
  std::cout << "  - Slack variables: " << slack_variables << std::endl;
  if (rewrite_equalities)
  {
    std::cout << "  - Determined variables: " << determined_variables << std::endl;
    std::cout << "  - Free variables: " << free_variables << std::endl;
  }
  else
  {
    std::cout << "  - Not using QR decomposition" << std::endl;
  }
  if (use_sparsity)
  {
    std::cout << "  - Using sparsity" << std::endl;
  }
  else
  {
    std::cout << "  - Not using sparsity" << std::endl;
  }
}
};  // namespace placo::problem