#include <map>
#include "placo/problem/problem.h"
#include "placo/problem/qp_error.h"
#include "eiquadprog/eiquadprog.hpp"

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

ProblemConstraints Problem::add_constraints(const std::vector<ProblemConstraint>& constraints)
{
  ProblemConstraints problem_constraints;
  for (auto& constraint : constraints)
  {
    problem_constraints.constraints.push_back(&add_constraint(constraint));
  }
  return problem_constraints;
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
    if (constraint->inequality)
    {
      constraint->is_active = false;
      if (constraint->priority == ProblemConstraint::Soft)
      {
        slack_variables += constraint->expression.rows();
      }
    }
    else
    {
      constraint->is_active = true;
    }
  }

  Eigen::MatrixXd P(n_variables + slack_variables, n_variables + slack_variables);
  Eigen::VectorXd q(n_variables + slack_variables);

  P.setZero();
  q.setZero();

  // Adding regularization
  // XXX: The user variables should maybe not be regularized by default?
  P.setIdentity();

  // Adding an epsilon regularization for variables (and no regularization fo slack variables)
  P.block(0, 0, n_variables, n_variables) *= 1e-8;
  P.block(n_variables, n_variables, slack_variables, slack_variables) *= 0;

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

    if (constraint->inequality)
    {
      // If the constraint is hard, this will be the true inequality, else, this will be the inequality
      // enforcing the slack variable to be >= 0
      n_inequalities += constraint->expression.rows();
    }
    else
    {
      if (constraint->priority == ProblemConstraint::Hard)
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
          }

          q.block(0, 0, constraint->expression.A.cols(), 1).noalias() +=
              constraint->weight * (constraint->expression.A.transpose() * constraint->expression.b);
        }
        else
        {
          int n = constraint->expression.A.cols();
          P.block(0, 0, n, n).noalias() +=
              constraint->weight * (constraint->expression.A.transpose() * constraint->expression.A);
          q.block(0, 0, n, 1).noalias() +=
              constraint->weight * (constraint->expression.A.transpose() * constraint->expression.b);
        }
      }
    }
  }

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

  // Used to keep track of the hard/soft inequalities constraints
  // The hard mapping maps index from inequality row to constraint, and the soft
  // mapping maps index from slack variables to the constraint.
  std::map<int, ProblemConstraint*> hard_inequalities_mapping;
  std::map<int, ProblemConstraint*> soft_inequalities_mapping;

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
      if (constraint->priority == ProblemConstraint::Hard)
      {
        // Ax + b >= 0
        G.block(k_inequality, 0, constraint->expression.rows(), constraint->expression.cols()) =
            constraint->expression.A;
        h.block(k_inequality, 0, constraint->expression.rows(), 1) = constraint->expression.b;

        for (int k = k_inequality; k < k_inequality + constraint->expression.rows(); k++)
        {
          hard_inequalities_mapping[k] = constraint;
        }
        k_inequality += constraint->expression.rows();
      }
      else
      {
        // min(Ax + b - s)
        // A slack variable is assigend with all "soft" inequality and a minimization is added to the problem
        Eigen::MatrixXd As(constraint->expression.rows(), n_variables + slack_variables);
        As.setZero();
        As.block(0, 0, constraint->expression.rows(), constraint->expression.cols()) = constraint->expression.A;

        for (int k = 0; k < constraint->expression.rows(); k++)
        {
          soft_inequalities_mapping[k_slack] = constraint;
          As(k, n_variables + k_slack) = -1;
          k_slack += 1;
        }

        P.noalias() += constraint->weight * (As.transpose() * As);
        q.noalias() += constraint->weight * (As.transpose() * constraint->expression.b);
      }
    }
    else if (constraint->priority == ProblemConstraint::Hard)
    {
      // Ax + b = 0
      A.block(k_equality, 0, constraint->expression.rows(), constraint->expression.cols()) = constraint->expression.A;
      b.block(k_equality, 0, constraint->expression.rows(), 1) = constraint->expression.b;
      k_equality += constraint->expression.rows();
    }
  }

  Eigen::VectorXi active_set;
  size_t active_set_size;

  x = Eigen::VectorXd(n_variables + slack_variables);
  x.setZero();
  double result =
      eiquadprog::solvers::solve_quadprog(P, q, A.transpose(), b, G.transpose(), h, x, active_set, active_set_size);

  // Checking that the problem is indeed feasible
  if (result == std::numeric_limits<double>::infinity())
  {
    throw QPError("Problem: Infeasible QP (check your hard inequality constraints)");
  }

  // Checking that equality constraints were enforced, since this is not covered by above result
  if (A.rows() > 0)
  {
    Eigen::VectorXd equality_constraints = A * x + b;
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

  slacks = x.block(n_variables, 0, slack_variables, 1);
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

};  // namespace placo