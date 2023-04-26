#include "placo/problem/problem.h"
#include "eiquadprog/eiquadprog.hpp"
#include "rhoban_utils/timing/time_stamp.h"

namespace placo
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
  variable->k_start = n_variables;
  variable->k_end = n_variables + size;
  n_variables += size;

  variables.push_back(variable);

  return *variable;
}

void Problem::add_limit(Expression expression, Eigen::VectorXd target)
{
  // -target <= expression <= target
  add_constraint(-target <= expression);
  add_constraint(expression <= target);
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

void Problem::solve()
{
  int n_equalities = 0;
  int n_inequalities = 0;
  int slack_variables = 0;

  for (auto constraint : constraints)
  {
    if (constraint->inequality && !constraint->hard)
    {
      slack_variables += constraint->expression.rows();
    }
  }

  Eigen::MatrixXd P(n_variables + slack_variables, n_variables + slack_variables);
  Eigen::VectorXd q(n_variables + slack_variables);

  P.setZero();
  q.setZero();

  // Adding regularization
  // XXX: The user variables should maybe not be regularized by default?
  P.setIdentity();
  P *= 1e-8;

  // rhoban_utils::TimeStamp t0 = rhoban_utils::TimeStamp::now();

  // Scanning the constraints (counting inequalities and equalities, building objectif function)
  for (auto constraint : constraints)
  {
    if (constraint->inequality)
    {
      // If the constraint is hard, this will be the true inequality, else, this will be the inequality
      // enforcing the slack variable to be >= 0
      n_inequalities += constraint->expression.rows();
    }
    else
    {
      if (constraint->hard)
      {
        n_equalities += constraint->expression.rows();
      }
      else
      {
        // Adding the soft constraint to the objective function
        if (use_sparsity)
        {
          Sparsity sparsity = Sparsity::detect_columns_sparsity(constraint->expression.A);

          int constraints = constraint->expression.A.rows();

          for (auto interval : sparsity.intervals)
          {
            int size = 1 + interval.end - interval.start;

            Eigen::MatrixXd block = constraint->expression.A.block(0, interval.start, constraints, size);

            P.block(interval.start, interval.start, size, size).noalias() +=
                constraint->weight * block.transpose() * block;
            q.noalias() += constraint->weight * (constraint->expression.A.transpose() * constraint->expression.b);
          }
        }
        else
        {
          P.noalias() += constraint->weight * (constraint->expression.A.transpose() * constraint->expression.A);
          q.noalias() += constraint->weight * (constraint->expression.A.transpose() * constraint->expression.b);
        }
      }
    }
  }

  // rhoban_utils::TimeStamp t1 = rhoban_utils::TimeStamp::now();
  // std::cout << "Hessian computation: " << (diffMs(t0, t1) * 1e3) << std::endl;

  // Equality constraints
  Eigen::MatrixXd A(n_equalities, n_variables + slack_variables);
  Eigen::VectorXd b(n_equalities);
  A.setZero();
  b.setZero();

  // Inequality constraints
  Eigen::MatrixXd G(n_inequalities, n_variables + slack_variables);
  Eigen::VectorXd h(n_inequalities);
  G.setZero();
  h.setZero();

  int k_equality = 0;
  int k_inequality = 0;
  int k_slack = 0;

  // Slack variables should be positive
  for (int slack = 0; slack < slack_variables; slack += 1)
  {
    // s_i >= 0
    G(k_inequality, n_variables + slack) = 1;
    k_inequality += 1;
  }

  for (auto constraint : constraints)
  {
    if (constraint->inequality)
    {
      if (constraint->hard)
      {
        // Ax + b >= 0
        G.block(k_inequality, 0, constraint->expression.rows(), constraint->expression.cols()) =
            constraint->expression.A;
        h.block(k_inequality, 0, constraint->expression.rows(), 1) = constraint->expression.b;
        k_inequality += constraint->expression.rows();
      }
      else
      {
        // min(Ax + b - s)
        // A slack variable is assigend with all "soft" inequality and a minimization is added to the problem
        Eigen::MatrixXd As(constraint->expression.rows(), n_variables + slack_variables);
        As.setZero();
        As.block(0, 0, As.rows(), n_variables) = constraint->expression.A;

        for (int k = 0; k < constraint->expression.rows(); k++)
        {
          As(k, n_variables + k_slack) = -1;
          k_slack += 1;
        }

        P.noalias() += constraint->weight * (As.transpose() * As);
        q.noalias() += constraint->weight * (As.transpose() * constraint->expression.b);
      }
    }
    else if (constraint->hard)
    {
      // Ax + b = 0
      A.block(k_equality, 0, constraint->expression.rows(), constraint->expression.cols()) = constraint->expression.A;
      b.block(k_equality, 0, constraint->expression.rows(), 1) = constraint->expression.b;
      k_equality += constraint->expression.rows();
    }
  }

  Eigen::VectorXi activeSet;
  size_t activeSetSize;

  Eigen::VectorXd x(n_variables + slack_variables);
  x.setZero();
  double result =
      eiquadprog::solvers::solve_quadprog(P, q, A.transpose(), b, G.transpose(), h, x, activeSet, activeSetSize);

  // Checking that the problem is indeed feasible
  if (result == std::numeric_limits<double>::infinity())
  {
    throw std::runtime_error("Problem: Infeasible QP (check your hard inequality constraints)");
  }

  // Checking that equality constraints were enforced, since this is not covered by above result
  if (A.rows() > 0)
  {
    Eigen::VectorXd equality_constraints = A * x + b;
    for (int k = 0; k < A.rows(); k++)
    {
      if (fabs(equality_constraints[k]) > 1e-8)
      {
        throw std::runtime_error("Problem: Infeasible QP (equality constraints were not enforced)");
      }
    }
  }

  // Checking for NaNs in solution
  if (x.hasNaN())
  {
    throw std::runtime_error("Problem: NaN in the QP solution");
  }

  slacks = x.block(n_variables, 0, slack_variables, 1);

  for (auto variable : variables)
  {
    variable->version += 1;
    variable->value = Eigen::VectorXd(variable->size());
    variable->value = x.block(variable->k_start, 0, variable->size(), 1);
  }
}

};  // namespace placo