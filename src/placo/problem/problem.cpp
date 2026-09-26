#include <iostream>
#include <chrono>
#include <algorithm>
#include "placo/problem/problem.h"
#include "placo/problem/qp_error.h"

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

void Problem::add_bounds(const Variable& variable, int start, const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper)
{
  if (lower.rows() != upper.rows() || start < 0 || variable.k_start + start + lower.rows() > variable.k_end)
  {
    throw QPError("Problem: invalid bounds size");
  }

  // Unbounded by default
  int bounded = lower_bounds.rows();
  if (bounded < n_variables)
  {
    lower_bounds.conservativeResize(n_variables);
    upper_bounds.conservativeResize(n_variables);
    lower_bounds.tail(n_variables - bounded).setConstant(-std::numeric_limits<double>::infinity());
    upper_bounds.tail(n_variables - bounded).setConstant(std::numeric_limits<double>::infinity());
  }

  auto lower_segment = lower_bounds.segment(variable.k_start + start, lower.rows());
  auto upper_segment = upper_bounds.segment(variable.k_start + start, upper.rows());
  lower_segment = lower_segment.cwiseMax(lower);
  upper_segment = upper_segment.cwiseMin(upper);
}

void Problem::detect_fixed_variables()
{
  fixed_variables = 0;
  fixed_values = Eigen::VectorXd::Zero(n_variables);
  unfixed_indices.resize(n_variables);
  for (int k = 0; k < n_variables; k++)
  {
    if (k < lower_bounds.rows() && std::isfinite(lower_bounds[k]) && lower_bounds[k] == upper_bounds[k])
    {
      fixed_values[k] = lower_bounds[k];
      fixed_variables += 1;
    }
    else
    {
      unfixed_indices[k - fixed_variables] = k;
    }
  }
  unfixed_indices.conservativeResize(n_variables - fixed_variables);
}

int Problem::bounds_inequalities() const
{
  return lower_bounds.array().isFinite().count() + upper_bounds.array().isFinite().count() - 2 * fixed_variables;
}

void Problem::bounded_values(std::vector<int>& bounded, Eigen::MatrixXd& A, Eigen::MatrixXd& b)
{
  bounded.clear();
  for (int k = 0; k < unfixed_indices.rows(); k++)
  {
    int index = unfixed_indices[k];
    if (index < lower_bounds.rows() && (std::isfinite(lower_bounds[index]) || std::isfinite(upper_bounds[index])))
    {
      bounded.push_back(k);
    }
  }

  if (determined_variables)
  {
    // With the QR elimination, x = Q [y; z]
    Eigen::MatrixXd full_A = Eigen::MatrixXd::Zero(bounded.size(), unfixed_indices.rows());
    for (int k = 0; k < (int)bounded.size(); k++)
    {
      full_A(k, bounded[k]) = 1;
    }
    QR.matrixQ().applyThisOnTheRight(full_A);
    A = full_A.rightCols(free_variables);
    b = full_A.leftCols(determined_variables) * y;
  }
  else
  {
    A = Eigen::MatrixXd::Zero(bounded.size(), free_variables);
    b = Eigen::MatrixXd::Zero(bounded.size(), 1);
    for (int k = 0; k < (int)bounded.size(); k++)
    {
      A(k, bounded[k]) = 1;
    }
  }
}

void Problem::clear_constraints()
{
  for (auto constraint : constraints)
  {
    delete constraint;
  }

  constraints.clear();
  lower_bounds.resize(0);
  upper_bounds.resize(0);
}

void Problem::clear_variables()
{
  for (auto variable : variables)
  {
    delete variable;
  }

  variables.clear();
  n_variables = 0;
  lower_bounds.resize(0);
  upper_bounds.resize(0);
}

void Problem::get_constraint_expressions(ProblemConstraint* constraint, Eigen::MatrixXd& A, Eigen::MatrixXd& b)
{
  const Eigen::MatrixXd* expression_A = &constraint->expression.A;
  b = constraint->expression.b;

  // Substituting the fixed variables: A x + b = A_unfixed x_unfixed + (b + A_fixed x_fixed)
  Eigen::MatrixXd unfixed_A;
  if (fixed_variables)
  {
    int cols = constraint->expression.A.cols();
    int unfixed_cols = std::lower_bound(unfixed_indices.data(), unfixed_indices.data() + unfixed_indices.rows(), cols) -
                       unfixed_indices.data();
    unfixed_A.resize(constraint->expression.A.rows(), unfixed_cols);
    for (int k = 0; k < unfixed_cols; k++)
    {
      unfixed_A.col(k) = constraint->expression.A.col(unfixed_indices[k]);
    }
    for (int k = 0; k < cols; k++)
    {
      if (fixed_values[k] != 0)
      {
        b += constraint->expression.A.col(k) * fixed_values[k];
      }
    }
    expression_A = &unfixed_A;
  }

  if (determined_variables)
  {
    Eigen::MatrixXd full_A(expression_A->rows(), unfixed_indices.rows());
    full_A.setZero();
    full_A.leftCols(expression_A->cols()) = *expression_A;
    QR.matrixQ().applyThisOnTheRight(full_A);

    A = full_A.rightCols(free_variables);
    b += full_A.leftCols(determined_variables) * y;
  }
  else if (fixed_variables)
  {
    A.swap(unfixed_A);
  }
  else
  {
    A = constraint->expression.A;
  }
}

void Problem::solve()
{
  n_equalities = 0;
  n_inequalities = 0;
  slack_variables = 0;
  determined_variables = 0;
  detect_fixed_variables();
  int unfixed_variables = unfixed_indices.rows();

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

  // Equality constraints (on the unfixed variables)
  Eigen::MatrixXd A(n_equalities, unfixed_variables);
  Eigen::VectorXd b(n_equalities);
  A.setZero();
  b.setZero();
  int k_equality = 0;

  for (auto constraint : constraints)
  {
    if (constraint->type == ProblemConstraint::Equality && constraint->priority == ProblemConstraint::Hard)
    {
      // Ax + b = 0
      Eigen::MatrixXd expression_A, expression_b;
      get_constraint_expressions(constraint, expression_A, expression_b);
      A.block(k_equality, 0, expression_A.rows(), expression_A.cols()) = expression_A;
      b.segment(k_equality, expression_b.rows()) = expression_b;
      k_equality += expression_b.rows();
    }
  }

  free_variables = unfixed_variables;

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

    free_variables = unfixed_variables - determined_variables;

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
  int hard_inequalities = 0;
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
      if (constraint->priority == ProblemConstraint::Hard)
      {
        hard_inequalities += constraint->expression.rows();
      }
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

        // All the (interval, interval) blocks of A^T A are added, including the cross terms between different
        // intervals
        for (auto interval_i : sparsity.intervals)
        {
          int size_i = 1 + interval_i.end - interval_i.start;
          for (auto interval_j : sparsity.intervals)
          {
            int size_j = 1 + interval_j.end - interval_j.start;
            P.block(interval_i.start, interval_j.start, size_i, size_j).noalias() +=
                constraint->weight * expression_A.middleCols(interval_i.start, size_i).transpose() *
                expression_A.middleCols(interval_j.start, size_j);
          }
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

  n_inequalities += bounds_inequalities();

  // The QP is solved with qpmad, in the variables z = [free variables, slack variables]:
  //   min 1/2 z^T P z + q^T z   subject to   lb <= z <= ub (simple bounds)   and   lower <= C z <= upper
  const double infinity = std::numeric_limits<double>::infinity();
  int n_qp = free_variables + slack_variables;

  // Bounds (see add_bounds), as a function of the QP variables. When no variable is eliminated, they are simple
  // bounds on the QP variables, else they are two-sided constraints (only one side can be active)
  std::vector<int> bounded;
  Eigen::MatrixXd bounded_A, bounded_b;
  bounded_values(bounded, bounded_A, bounded_b);
  bool simple_bounds = (determined_variables == 0);

  // Simple bounds, including the positivity of slack variables
  Eigen::VectorXd lb, ub;
  if (slack_variables > 0 || (simple_bounds && !bounded.empty()))
  {
    lb = Eigen::VectorXd::Constant(n_qp, -infinity);
    ub = Eigen::VectorXd::Constant(n_qp, infinity);
    lb.tail(slack_variables).setZero();
  }

  // General constraints: equalities (if they are not eliminated), hard inequalities and bounds that are not simple
  int n_bound_rows = simple_bounds ? 0 : bounded.size();
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(A.rows() + hard_inequalities + n_bound_rows, n_qp);
  Eigen::VectorXd lower(C.rows());
  Eigen::VectorXd upper(C.rows());

  // Ax + b = 0
  C.topLeftCorner(A.rows(), A.cols()) = A;
  lower.head(A.rows()) = -b;
  upper.head(A.rows()) = -b;

  // Used to keep track of the hard/soft inequalities constraints
  // The hard mapping maps index from general constraint row to constraint, and the soft
  // mapping maps index from slack variables to the constraint.
  std::vector<ProblemConstraint*> hard_inequalities_mapping(C.rows(), nullptr);
  std::vector<ProblemConstraint*> soft_inequalities_mapping(slack_variables, nullptr);

  int row = A.rows();
  int k_slack = 0;

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
        C.block(row, 0, expression_A.rows(), expression_A.cols()) = expression_A;
        lower.segment(row, expression_b.rows()) = -expression_b;
        upper.segment(row, expression_b.rows()).setConstant(infinity);

        for (int k = row; k < row + expression_A.rows(); k++)
        {
          hard_inequalities_mapping[k] = constraint;
        }
        row += expression_A.rows();
      }
      else
      {
        // min ||Ax + b - s||^2, with a slack variable s >= 0 assigned to each row of the soft inequality.
        // With As = [A, -I] (I on this constraint's own slack columns), As^T As only has three non-zero blocks, which
        // are updated directly instead of building the full-width As (that would cost O(rows (n + slacks)^2)).
        int rows = expression_A.rows(), cols = expression_A.cols();
        int s = free_variables + k_slack;
        double w = constraint->weight;

        P.block(0, 0, cols, cols).noalias() += w * expression_A.transpose() * expression_A;
        P.block(0, s, cols, rows).noalias() -= w * expression_A.transpose();
        P.block(s, 0, rows, cols).noalias() -= w * expression_A;
        P.block(s, s, rows, rows).diagonal().array() += w;

        q.segment(0, cols).noalias() += w * expression_A.transpose() * expression_b;
        q.segment(s, rows).noalias() -= w * expression_b;

        for (int k = 0; k < rows; k++)
        {
          soft_inequalities_mapping[k_slack] = constraint;
          k_slack += 1;
        }
      }
    }
  }

  // lower <= x <= upper, with x = bounded_A z + bounded_b
  for (int k = 0; k < (int)bounded.size(); k++)
  {
    double lower_k = lower_bounds[unfixed_indices[bounded[k]]] - bounded_b(k, 0);
    double upper_k = upper_bounds[unfixed_indices[bounded[k]]] - bounded_b(k, 0);
    if (simple_bounds)
    {
      lb[bounded[k]] = lower_k;
      ub[bounded[k]] = upper_k;
    }
    else
    {
      C.block(row, 0, 1, free_variables) = bounded_A.row(k);
      lower[row] = lower_k;
      upper[row] = upper_k;
      row += 1;
    }
  }

  // Constraint rows are normalized: qpmad uses absolute tolerances, which would else depend on the scale of each
  // constraint (the feasible set is unchanged)
  for (int k = 0; k < C.rows(); k++)
  {
    double norm = C.row(k).norm();
    if (norm > 0)
    {
      C.row(k) /= norm;
      lower[k] /= norm;
      upper[k] /= norm;
    }
  }

  // Solving the QP (P is factorized in place)
  Eigen::VectorXd qp_x(n_qp);
  bool feasible;
  if (n_qp == 0)
  {
    // All the variables are determined by the equalities, the remaining constraints are constants (0 <= upper and
    // lower <= 0)
    feasible = (lower.array() <= 1e-9).all() && (upper.array() >= -1e-9).all();
  }
  else
  {
    try
    {
      feasible = (qp_solver.solve(qp_x, P, q, lb, ub, C, lower, upper) == qpmad::Solver::OK);
    }
    catch (const std::exception& e)
    {
      feasible = false;
    }
  }

  Eigen::VectorXd unfixed_x;
  if (determined_variables)
  {
    unfixed_x = Eigen::VectorXd::Zero(unfixed_variables);
    unfixed_x.topRows(determined_variables) = y;
    unfixed_x.bottomRows(free_variables) = qp_x.topRows(free_variables);
    QR.matrixQ().applyThisOnTheLeft(unfixed_x);
  }
  else
  {
    unfixed_x = qp_x.topRows(free_variables);
  }

  x = fixed_values;
  for (int k = 0; k < unfixed_variables; k++)
  {
    x[unfixed_indices[k]] = unfixed_x[k];
  }

  // Checking that the problem is indeed feasible
  if (!feasible)
  {
    throw QPError("Problem: Infeasible QP (check your hard inequality constraints)");
  }

  // Checking that equality constraints were enforced, since this is not covered by above result
  if (A.rows() > 0)
  {
    Eigen::VectorXd equality_constraints = A * unfixed_x + b;
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

  // Reporting on the active constraints (indices of the active inequalities are the simple bounds, then the
  // general constraints rows)
  Eigen::VectorXd dual;
  Eigen::Matrix<qpmad::MatrixIndex, Eigen::Dynamic, 1> active_indices;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> active_is_lower;
  if (n_qp > 0)
  {
    qp_solver.getInequalityDual(dual, active_indices, active_is_lower);
  }
  for (int k = 0; k < active_indices.rows(); k++)
  {
    int row = active_indices[k] - lb.rows();
    if (row >= 0 && hard_inequalities_mapping[row] != nullptr)
    {
      hard_inequalities_mapping[row]->is_active = true;
    }
  }

  slacks = qp_x.block(free_variables, 0, slack_variables, 1);
  for (int k = 0; k < slacks.rows(); k++)
  {
    if (slacks[k] <= 1e-6 && soft_inequalities_mapping[k] != nullptr)
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
  std::cout << "  - Fixed variables: " << fixed_variables << std::endl;
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