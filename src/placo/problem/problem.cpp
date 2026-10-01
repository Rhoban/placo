#include <iostream>
#include <chrono>
#include <algorithm>
#include <functional>
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

  for (auto constraint : constraints_pool)
  {
    delete constraint;
  }

  for (auto variable : variables)
  {
    delete variable;
  }

  constraints.clear();
  constraints_pool.clear();
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

ProblemConstraint& Problem::add_constraint()
{
  ProblemConstraint* constraint;
  if (constraints_pool.empty())
  {
    constraint = new ProblemConstraint;
  }
  else
  {
    // Reusing a cleared constraint (and the memory of its matrices)
    constraint = constraints_pool.back();
    constraints_pool.pop_back();
    constraint->type = ProblemConstraint::Equality;
    constraint->priority = ProblemConstraint::Hard;
    constraint->weight = 1.0;
    constraint->is_active = false;
    constraint->columns.clear();
  }
  constraints.push_back(constraint);

  return *constraint;
}

ProblemConstraint& Problem::add_constraint(const ProblemConstraint& constraint_)
{
  ProblemConstraint& constraint = add_constraint();
  constraint = constraint_;

  return constraint;
}

void Problem::add_bounds(const Variable& variable, int start, const Eigen::Ref<const Eigen::VectorXd>& lower,
                         const Eigen::Ref<const Eigen::VectorXd>& upper)
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
  fixed_values.setZero(n_variables);
  unfixed_indices.clear();
  unfixed_index.resize(n_variables);
  for (int k = 0; k < n_variables; k++)
  {
    if (k < lower_bounds.rows() && std::isfinite(lower_bounds[k]) && lower_bounds[k] == upper_bounds[k])
    {
      fixed_values[k] = lower_bounds[k];
      unfixed_index[k] = -1;
      fixed_variables += 1;
    }
    else
    {
      unfixed_index[k] = k - fixed_variables;
      unfixed_indices.push_back(k);
    }
  }
}

int Problem::bounds_inequalities() const
{
  return lower_bounds.array().isFinite().count() + upper_bounds.array().isFinite().count() - 2 * fixed_variables;
}

void Problem::clear_constraints()
{
  // Constraints are kept to be reused, in reverse order so that they are reused in the same order
  constraints_pool.insert(constraints_pool.end(), constraints.rbegin(), constraints.rend());
  constraints.clear();

  // Bounds are reset (keeping their memory)
  lower_bounds.setConstant(-std::numeric_limits<double>::infinity());
  upper_bounds.setConstant(std::numeric_limits<double>::infinity());
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

void Problem::reduce_fixed(const ProblemConstraint& constraint, Reduced& r)
{
  const Eigen::MatrixXd& A = constraint.expression.A;
  int rows = A.rows(), cols = A.cols();
  r.b = constraint.expression.b;

  if (constraint.columns.empty())
  {
    // Dense expression: unfixed variables keep their order, the reduced expression is dense as well
    int unfixed_cols = std::lower_bound(unfixed_indices.begin(), unfixed_indices.end(), cols) - unfixed_indices.begin();
    r.columns = nullptr;
    if (unfixed_cols == cols)
    {
      new (&r.A) Eigen::Map<const Eigen::MatrixXd>(A.data(), rows, cols);
      return;
    }
    r.A_fixed.resize(rows, unfixed_cols);
    for (int k = 0; k < unfixed_cols; k++)
    {
      r.A_fixed.col(k) = A.col(unfixed_indices[k]);
    }
    for (int k = 0; k < cols; k++)
    {
      if (fixed_values[k] != 0)
      {
        r.b.noalias() += A.col(k) * fixed_values[k];
      }
    }
    new (&r.A) Eigen::Map<const Eigen::MatrixXd>(r.A_fixed.data(), rows, unfixed_cols);
    return;
  }

  // Compact expression
  const std::vector<int>& columns = constraint.columns;
  if (fixed_variables == 0)
  {
    new (&r.A) Eigen::Map<const Eigen::MatrixXd>(A.data(), rows, cols);
    r.columns = &columns;
    return;
  }

  int kept = 0;
  for (int column : columns)
  {
    kept += (unfixed_index[column] >= 0);
  }
  r.columns_fixed.resize(kept);
  if (kept == cols)
  {
    for (int k = 0; k < cols; k++)
    {
      r.columns_fixed[k] = unfixed_index[columns[k]];
    }
    new (&r.A) Eigen::Map<const Eigen::MatrixXd>(A.data(), rows, cols);
  }
  else
  {
    r.A_fixed.resize(rows, kept);
    int k_kept = 0;
    for (int k = 0; k < cols; k++)
    {
      int index = unfixed_index[columns[k]];
      if (index >= 0)
      {
        r.columns_fixed[k_kept] = index;
        r.A_fixed.col(k_kept++) = A.col(k);
      }
      else if (fixed_values[columns[k]] != 0)
      {
        r.b.noalias() += A.col(k) * fixed_values[columns[k]];
      }
    }
    new (&r.A) Eigen::Map<const Eigen::MatrixXd>(r.A_fixed.data(), rows, kept);
  }
  r.columns = &r.columns_fixed;
}

void Problem::reduce_eliminated(Reduced& r)
{
  if (determined_variables == 0)
  {
    return;
  }

  int rows = r.A.rows();

  if (!sparse_elimination_used)
  {
    // Dense QR elimination, already done for all the constraints by reduce_eliminated_qr
    return;
  }

  // Sparse elimination: x_free = z and x_eliminated = Z z + x0, z being the QP variables. The reduced expression is
  // compact, on the free variables it depends on (directly, or through the eliminated variables)
  int cols = r.A.cols();
  auto column_variable = [&](int k) { return r.columns == nullptr ? k : (*r.columns)[k]; };
  const Eigen::MatrixXd& Z = elimination.Z;

  stamp.resize(free_variables);
  std::fill(stamp.begin(), stamp.end(), -1);
  gathered.clear();
  for (int k = 0; k < cols; k++)
  {
    int variable = column_variable(k);
    if (free_column[variable] >= 0)
    {
      if (stamp[free_column[variable]] < 0)
      {
        stamp[free_column[variable]] = 0;
        gathered.push_back(free_column[variable]);
      }
    }
    else
    {
      for (int c : elimination.Z_columns[eliminated_row[variable]])
      {
        if (stamp[c] < 0)
        {
          stamp[c] = 0;
          gathered.push_back(c);
        }
      }
    }
  }
  std::sort(gathered.begin(), gathered.end());
  for (int k = 0; k < (int)gathered.size(); k++)
  {
    stamp[gathered[k]] = k;
  }

  r.columns_eliminated = gathered;
  r.A_eliminated.setZero(rows, gathered.size());
  for (int k = 0; k < cols; k++)
  {
    int variable = column_variable(k);
    if (free_column[variable] >= 0)
    {
      r.A_eliminated.col(stamp[free_column[variable]]) += r.A.col(k);
    }
    else
    {
      int row = eliminated_row[variable];
      for (int c : elimination.Z_columns[row])
      {
        r.A_eliminated.col(stamp[c]) += Z(row, c) * r.A.col(k);
      }
      r.b.noalias() += elimination.x0[row] * r.A.col(k);
    }
  }

  new (&r.A) Eigen::Map<const Eigen::MatrixXd>(r.A_eliminated.data(), rows, gathered.size());
  r.columns = &r.columns_eliminated;
}

void Problem::reduce_eliminated_qr()
{
  // Dense QR elimination: x = Q [y; z], z being the QP variables
  int n = (int)unfixed_indices.size();
  auto eliminated = [&](int i) {
    return constraints[i]->type == ProblemConstraint::Equality && constraints[i]->priority == ProblemConstraint::Hard;
  };

  int total_rows = 0;
  for (int i = 0; i < (int)constraints.size(); i++)
  {
    if (!eliminated(i))
    {
      total_rows += reduced[i].A.rows();
    }
  }

  stacked.setZero(total_rows, n);
  int offset = 0;
  for (int i = 0; i < (int)constraints.size(); i++)
  {
    if (eliminated(i))
    {
      continue;
    }
    Reduced& r = reduced[i];
    int rows = r.A.rows();
    if (r.columns == nullptr)
    {
      stacked.block(offset, 0, rows, r.A.cols()) = r.A;
    }
    else
    {
      for (int k = 0; k < (int)r.columns->size(); k++)
      {
        stacked.block(offset, (*r.columns)[k], rows, 1) = r.A.col(k);
      }
    }
    offset += rows;
  }

  QR.matrixQ().applyThisOnTheRight(stacked);

  offset = 0;
  for (int i = 0; i < (int)constraints.size(); i++)
  {
    if (eliminated(i))
    {
      continue;
    }
    Reduced& r = reduced[i];
    int rows = r.A.rows();
    r.b.noalias() += stacked.block(offset, 0, rows, determined_variables) * y;

    // The QP variables are the last columns (copied to be contiguous in memory)
    r.A_eliminated = stacked.block(offset, determined_variables, rows, free_variables);
    new (&r.A) Eigen::Map<const Eigen::MatrixXd>(r.A_eliminated.data(), rows, free_variables);
    r.columns = nullptr;
    offset += rows;
  }
}

void Problem::compute_runs(const Reduced& r, std::vector<Run>& runs, bool detect_zeros)
{
  runs.clear();
  int cols = r.A.cols();

  if (r.columns != nullptr)
  {
    // Consecutive variables are grouped
    for (int k = 0; k < cols; k++)
    {
      int variable = (*r.columns)[k];
      if (!runs.empty() && runs.back().var + runs.back().size == variable)
      {
        runs.back().size += 1;
      }
      else
      {
        runs.push_back(Run{ variable, k, 1 });
      }
    }
    return;
  }

  if (!detect_zeros)
  {
    if (cols > 0)
    {
      runs.push_back(Run{ 0, 0, cols });
    }
    return;
  }

  // Dense expression, skipping the columns that are zero
  for (int k = 0; k < cols; k++)
  {
    if (!r.A.col(k).isZero(0))
    {
      if (!runs.empty() && runs.back().col + runs.back().size == k)
      {
        runs.back().size += 1;
      }
      else
      {
        runs.push_back(Run{ k, k, 1 });
      }
    }
  }
}

void Problem::add_squared_norm(const Reduced& r, const std::vector<Run>& runs, double weight)
{
  // Only the lower triangle of P is built (it is the only part used by the QP solver)
  if (runs.size() == 1)
  {
    const Run& run = runs[0];
    auto A = r.A.middleCols(run.col, run.size);
    P.block(run.var, run.var, run.size, run.size).selfadjointView<Eigen::Lower>().rankUpdate(A.transpose(), weight);
    q.segment(run.var, run.size).noalias() += weight * A.transpose() * r.b;
    return;
  }

  bool compact = (r.columns != nullptr);
  if (compact)
  {
    // One product A^T A for all the columns, then scattered in P
    int cols = r.A.cols();
    gram.setZero(cols, cols);
    gram.selfadjointView<Eigen::Lower>().rankUpdate(r.A.transpose(), weight);
    for (int i = 0; i < (int)runs.size(); i++)
    {
      const Run& run_i = runs[i];
      for (int j = 0; j <= i; j++)
      {
        const Run& run_j = runs[j];
        P.block(run_i.var, run_j.var, run_i.size, run_j.size) +=
            gram.block(run_i.col, run_j.col, run_i.size, run_j.size);
      }
    }
  }
  else
  {
    // Dense expression with zero columns: one product per pair of runs
    for (int i = 0; i < (int)runs.size(); i++)
    {
      const Run& run_i = runs[i];
      auto A_i = r.A.middleCols(run_i.col, run_i.size);
      P.block(run_i.var, run_i.var, run_i.size, run_i.size)
          .selfadjointView<Eigen::Lower>()
          .rankUpdate(A_i.transpose(), weight);
      for (int j = 0; j < i; j++)
      {
        const Run& run_j = runs[j];
        P.block(run_i.var, run_j.var, run_i.size, run_j.size).noalias() +=
            weight * A_i.transpose() * r.A.middleCols(run_j.col, run_j.size);
      }
    }
  }

  for (const Run& run : runs)
  {
    q.segment(run.var, run.size).noalias() += weight * r.A.middleCols(run.col, run.size).transpose() * r.b;
  }
}

#ifndef PLACO_WITH_DAQP
void Problem::solve_daqp()
{
  throw QPError("Problem: placo was built without DAQP (PLACO_WITH_DAQP)");
}
#endif

void Problem::solve()
{
  if (backend == Backend::Daqp)
  {
    solve_daqp();
    return;
  }

  n_equalities = 0;
  n_inequalities = 0;
  slack_variables = 0;
  determined_variables = 0;
  sparse_elimination_used = false;
  detect_fixed_variables();
  int unfixed_variables = (int)unfixed_indices.size();
  const double infinity = std::numeric_limits<double>::infinity();

  // Checking and counting the constraints, substituting the fixed variables
  int hard_inequalities = 0;
  reduced.resize(constraints.size());
  for (int i = 0; i < (int)constraints.size(); i++)
  {
    ProblemConstraint* constraint = constraints[i];
    const Expression& e = constraint->expression;

    if (e.A.rows() == 0 || e.b.rows() == 0)
    {
      throw QPError("Problem: A or b is empty");
    }
    if (e.A.rows() != e.b.rows())
    {
      throw QPError("Problem: A.rows() != b.rows()");
    }
    if (constraint->columns.empty())
    {
      if (e.A.cols() > n_variables)
      {
        throw QPError("Problem: Inconsistent problem size");
      }
    }
    else if ((int)constraint->columns.size() != e.A.cols() || constraint->columns.back() >= n_variables ||
             constraint->columns.front() < 0 ||
             std::adjacent_find(constraint->columns.begin(), constraint->columns.end(),
                                std::greater_equal<int>()) != constraint->columns.end())
    {
      throw QPError("Problem: Inconsistent compact constraint columns");
    }

    if (constraint->type == ProblemConstraint::Inequality)
    {
      constraint->is_active = false;
      n_inequalities += e.rows();
      if (constraint->priority == ProblemConstraint::Soft)
      {
        slack_variables += e.rows();
      }
      else
      {
        hard_inequalities += e.rows();
      }
    }
    else
    {
      if (constraint->priority == ProblemConstraint::Hard)
      {
        n_equalities += e.rows();
      }
      constraint->is_active = true;
    }

    reduce_fixed(*constraint, reduced[i]);
  }

  free_variables = unfixed_variables;

  // Elimination of the hard equalities
  bool eliminate = rewrite_equalities && n_equalities > 0;
  if (eliminate && sparse_elimination)
  {
    std::vector<SparseElimination::Equality> equalities;
    for (int i = 0; i < (int)constraints.size(); i++)
    {
      ProblemConstraint* constraint = constraints[i];
      if (constraint->type == ProblemConstraint::Equality && constraint->priority == ProblemConstraint::Hard)
      {
        Reduced& r = reduced[i];
        if (r.columns == nullptr)
        {
          // Dense equalities are passed as compact on their non-zero columns (all the columns if use_sparsity is not
          // set), the storage for eliminated expressions being unused for equalities
          r.columns_eliminated.clear();
          for (int k = 0; k < r.A.cols(); k++)
          {
            if (!use_sparsity || !r.A.col(k).isZero(0))
            {
              r.columns_eliminated.push_back(k);
            }
          }
          r.A_eliminated.resize(r.A.rows(), r.columns_eliminated.size());
          for (int k = 0; k < (int)r.columns_eliminated.size(); k++)
          {
            r.A_eliminated.col(k) = r.A.col(r.columns_eliminated[k]);
          }
          new (&r.A) Eigen::Map<const Eigen::MatrixXd>(r.A_eliminated.data(), r.A_eliminated.rows(),
                                                       r.A_eliminated.cols());
          r.columns = &r.columns_eliminated;
        }
        equalities.push_back(SparseElimination::Equality{ &r.A, &r.b, r.columns });
      }
    }
    sparse_elimination_used = elimination.eliminate(unfixed_variables, equalities);

    if (sparse_elimination_used)
    {
      determined_variables = elimination.pivots.size();
      free_variables = elimination.free.size();
      free_column.assign(unfixed_variables, -1);
      eliminated_row.assign(unfixed_variables, -1);
      for (int k = 0; k < free_variables; k++)
      {
        free_column[elimination.free[k]] = k;
      }
      for (int k = 0; k < determined_variables; k++)
      {
        eliminated_row[elimination.pivots[k]] = k;
      }
    }
  }

  // Equalities matrix (Aeq x + beq = 0), for the QR elimination or passed to the QP solver
  int equality_rows = 0;
  if (!sparse_elimination_used && n_equalities > 0)
  {
    Aeq.setZero(n_equalities, unfixed_variables);
    beq.resize(n_equalities);
    for (int i = 0; i < (int)constraints.size(); i++)
    {
      ProblemConstraint* constraint = constraints[i];
      if (constraint->type == ProblemConstraint::Equality && constraint->priority == ProblemConstraint::Hard)
      {
        const Reduced& r = reduced[i];
        int rows = r.A.rows();
        if (r.columns == nullptr)
        {
          Aeq.block(equality_rows, 0, rows, r.A.cols()) = r.A;
        }
        else
        {
          for (int k = 0; k < (int)r.columns->size(); k++)
          {
            Aeq.block(equality_rows, (*r.columns)[k], rows, 1) = r.A.col(k);
          }
        }
        beq.segment(equality_rows, rows) = r.b;
        equality_rows += rows;
      }
    }
  }

  if (eliminate && !sparse_elimination_used)
  {
    // Computing QR decomposition of A.T
    QR.compute(Aeq.transpose());

    determined_variables = QR.rank();

    if (determined_variables != Aeq.rows())
    {
      throw QPError("QR decomposition failed to find a full rank matrix for equality constraints");
    }

    R = QR.matrixR().transpose().block(0, 0, determined_variables, determined_variables);
    b2 = beq.transpose();
    QR.colsPermutation().applyThisOnTheRight(b2);
    b2.transposeInPlace();

    y = R.triangularView<Eigen::Lower>().solve(-b2);

    free_variables = unfixed_variables - determined_variables;
  }

  // Equalities that are not eliminated are passed to the QP solver
  if (eliminate)
  {
    equality_rows = 0;
    n_equalities = 0;
  }

  // The QP is solved with qpmad, in the variables z = [free variables, slack variables]:
  //   min 1/2 z^T P z + q^T z   subject to   lb <= z <= ub (simple bounds)   and   lower <= C z <= upper
  int n_qp = free_variables + slack_variables;

  P.setZero(n_qp, n_qp);
  q.setZero(n_qp);

  // Adding regularization
  P.diagonal().head(free_variables).setConstant(regularization);
  if (sparse_elimination_used)
  {
    // The regularization applies to all the variables (the eliminated ones being Z z + x0), which is the same as with
    // the QR elimination (up to a constant)
    const Eigen::MatrixXd& Z = elimination.Z;
    P.topLeftCorner(free_variables, free_variables)
        .selfadjointView<Eigen::Lower>()
        .rankUpdate(Z.transpose(), regularization);
    q.head(free_variables).noalias() += regularization * Z.transpose() * elimination.x0;
  }

  // Bounds on the unfixed variables, as a function of the QP variables. Bounds on free variables are simple bounds
  // (all of them without elimination), the others become two-sided general constraints (only one side can be active)
  n_inequalities += bounds_inequalities();
  int simple_bounds = 0, bound_rows = 0;
  for (int k = 0; k < unfixed_variables; k++)
  {
    int index = unfixed_indices[k];
    if (index < lower_bounds.rows() && (std::isfinite(lower_bounds[index]) || std::isfinite(upper_bounds[index])))
    {
      if (determined_variables == 0 || (sparse_elimination_used && free_column[k] >= 0))
      {
        simple_bounds += 1;
      }
      else
      {
        bound_rows += 1;
      }
    }
  }

  // Simple bounds, including the positivity of slack variables
  bool has_simple_bounds = slack_variables > 0 || simple_bounds > 0;
  if (has_simple_bounds)
  {
    lb.setConstant(n_qp, -infinity);
    ub.setConstant(n_qp, infinity);
    lb.tail(slack_variables).setZero();
  }
  else
  {
    lb.resize(0);
    ub.resize(0);
  }

  // General constraints: equalities (if they are not eliminated), hard inequalities and bounds that are not simple
  int n_rows = equality_rows + hard_inequalities + bound_rows;
  C.setZero(n_rows, n_qp);
  lower.resize(n_rows);
  upper.resize(n_rows);

  // Used to keep track of the hard/soft inequalities constraints
  // The hard mapping maps index from general constraint row to constraint, and the soft
  // mapping maps index from slack variables to the constraint.
  hard_inequalities_mapping.assign(n_rows, nullptr);
  soft_inequalities_mapping.assign(slack_variables, nullptr);

  if (eliminate && !sparse_elimination_used && determined_variables > 0)
  {
    reduce_eliminated_qr();
  }

  // Filling the objective and the general constraints
  int row = 0;
  int k_slack = 0;
  for (int i = 0; i < (int)constraints.size(); i++)
  {
    ProblemConstraint* constraint = constraints[i];
    Reduced& r = reduced[i];
    bool hard_equality =
        constraint->type == ProblemConstraint::Equality && constraint->priority == ProblemConstraint::Hard;

    if (hard_equality && eliminate)
    {
      // Eliminated
      continue;
    }

    reduce_eliminated(r);
    int rows = r.A.rows();

    if (constraint->priority == ProblemConstraint::Soft)
    {
      compute_runs(r, runs, use_sparsity);
      add_squared_norm(r, runs, constraint->weight);

      if (constraint->type == ProblemConstraint::Inequality)
      {
        // min ||Ax + b - s||^2, with a slack variable s >= 0 assigned to each row of the soft inequality: the cross
        // terms -A^T s are added (the ||s||^2 terms on the diagonal)
        int s = free_variables + k_slack;
        double w = constraint->weight;
        for (const Run& run : runs)
        {
          P.block(s, run.var, rows, run.size).noalias() -= w * r.A.middleCols(run.col, run.size);
        }
        P.block(s, s, rows, rows).diagonal().array() += w;
        q.segment(s, rows).noalias() -= w * r.b;

        for (int k = 0; k < rows; k++)
        {
          soft_inequalities_mapping[k_slack] = constraint;
          k_slack += 1;
        }
      }
    }
    else
    {
      // Ax + b = 0 or Ax + b >= 0, as a general constraint
      compute_runs(r, runs, false);
      for (const Run& run : runs)
      {
        C.block(row, run.var, rows, run.size) = r.A.middleCols(run.col, run.size);
      }
      lower.segment(row, rows) = -r.b;
      if (hard_equality)
      {
        upper.segment(row, rows) = -r.b;
      }
      else
      {
        upper.segment(row, rows).setConstant(infinity);
        for (int k = row; k < row + rows; k++)
        {
          hard_inequalities_mapping[k] = constraint;
        }
      }
      row += rows;
    }
  }

  // lower <= x <= upper for the bounded unfixed variables
  if (simple_bounds + bound_rows > 0)
  {
    bool qr_bounds = determined_variables > 0 && !sparse_elimination_used;
    if (qr_bounds)
    {
      // With the QR elimination, x = Q [y; z]
      bounds_full.setZero(bound_rows, unfixed_variables);
      int k_row = 0;
      for (int k = 0; k < unfixed_variables; k++)
      {
        int index = unfixed_indices[k];
        if (index < lower_bounds.rows() && (std::isfinite(lower_bounds[index]) || std::isfinite(upper_bounds[index])))
        {
          bounds_full(k_row++, k) = 1;
        }
      }
      QR.matrixQ().applyThisOnTheRight(bounds_full);
    }

    int k_row = 0;
    for (int k = 0; k < unfixed_variables; k++)
    {
      int index = unfixed_indices[k];
      if (index >= lower_bounds.rows() || !(std::isfinite(lower_bounds[index]) || std::isfinite(upper_bounds[index])))
      {
        continue;
      }

      if (determined_variables == 0 || (sparse_elimination_used && free_column[k] >= 0))
      {
        int qp_variable = determined_variables == 0 ? k : free_column[k];
        lb[qp_variable] = lower_bounds[index];
        ub[qp_variable] = upper_bounds[index];
      }
      else
      {
        double offset;
        if (qr_bounds)
        {
          C.block(row, 0, 1, free_variables) = bounds_full.block(k_row, determined_variables, 1, free_variables);
          offset = bounds_full.row(k_row).head(determined_variables).dot(y.col(0));
          k_row += 1;
        }
        else
        {
          int z_row = eliminated_row[k];
          for (int c : elimination.Z_columns[z_row])
          {
            C(row, c) = elimination.Z(z_row, c);
          }
          offset = elimination.x0[z_row];
        }
        lower[row] = lower_bounds[index] - offset;
        upper[row] = upper_bounds[index] - offset;
        row += 1;
      }
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
  qp_x.resize(n_qp);
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

    // The solution is checked against the constraints: on some infeasible problems, the iterations of the solver
    // diverge to huge values, where rounding errors exceed its (absolute) tolerances, and it can then report a success
    if (feasible)
    {
      auto violated = [](double value, double lower_value, double upper_value) {
        return value < lower_value - 1e-6 * (1 + fabs(lower_value)) || value > upper_value + 1e-6 * (1 + fabs(upper_value));
      };
      Eigen::VectorXd Cz = C * qp_x;
      for (int k = 0; k < C.rows() && feasible; k++)
      {
        feasible = !violated(Cz[k], lower[k], upper[k]);
      }
      for (int k = 0; k < lb.rows() && feasible; k++)
      {
        feasible = !violated(qp_x[k], lb[k], ub[k]);
      }
    }
  }

  // Values of the unfixed variables
  if (sparse_elimination_used)
  {
    unfixed_x.resize(unfixed_variables);
    for (int k = 0; k < free_variables; k++)
    {
      unfixed_x[elimination.free[k]] = qp_x[k];
    }
    Eigen::VectorXd eliminated = elimination.Z * qp_x.head(free_variables) + elimination.x0;
    for (int k = 0; k < determined_variables; k++)
    {
      unfixed_x[elimination.pivots[k]] = eliminated[k];
    }
  }
  else if (determined_variables)
  {
    unfixed_x.setZero(unfixed_variables);
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
  if (equality_rows > 0)
  {
    Eigen::VectorXd equality_constraints = Aeq * unfixed_x + beq;
    for (int k = 0; k < equality_rows; k++)
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
  if (n_qp > 0)
  {
    Eigen::VectorXd dual;
    Eigen::Matrix<qpmad::MatrixIndex, Eigen::Dynamic, 1> active_indices;
    Eigen::Matrix<bool, Eigen::Dynamic, 1> active_is_lower;
    qp_solver.getInequalityDual(dual, active_indices, active_is_lower);
    for (int k = 0; k < active_indices.rows(); k++)
    {
      int active_row = active_indices[k] - lb.rows();
      if (active_row >= 0 && hard_inequalities_mapping[active_row] != nullptr)
      {
        hard_inequalities_mapping[active_row]->is_active = true;
      }
    }
  }

  slacks = qp_x.segment(free_variables, slack_variables);
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
    variable->value = x.segment(variable->k_start, variable->size());
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
    if (sparse_elimination_used)
    {
      std::cout << "  - Using sparse elimination of equalities" << std::endl;
    }
  }
  else
  {
    std::cout << "  - Not eliminating equalities" << std::endl;
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
}  // namespace placo::problem
