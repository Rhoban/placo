#include <algorithm>
#include <cmath>
#include <daqp/api.h>
#include "placo/problem/problem.h"
#include "placo/problem/qp_error.h"

namespace placo::problem
{
void Problem::solve_daqp()
{
  if (!(daqp_soft_l1_weight >= 0) || !std::isfinite(daqp_soft_l1_weight))
  {
    throw QPError("Problem: invalid daqp_soft_l1_weight");
  }

  n_equalities = 0;
  slack_variables = 0;
  determined_variables = 0;
  sparse_elimination_used = false;
  detect_fixed_variables();
  const int n = (int)unfixed_indices.size();
  free_variables = n;
  n_inequalities = bounds_inequalities();

  // Checking the constraints, substituting the fixed variables, and counting the DAQP rows (the soft equalities are
  // added to the objective)
  int rows = 0;
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

    const bool soft = constraint->priority == ProblemConstraint::Soft;
    if (soft && constraint->type == ProblemConstraint::Inequality &&
        (!(constraint->weight > 0) || !std::isfinite(constraint->weight)))
    {
      throw QPError("Problem: soft inequalities need a positive weight with DAQP");
    }

    constraint->is_active = constraint->type == ProblemConstraint::Equality;
    if (!soft || constraint->type == ProblemConstraint::Inequality)
    {
      rows += e.rows();
    }
    reduce_fixed(*constraint, reduced[i]);
  }

  // The bounds of the unfixed variables are DAQP simple bounds (the first ms constraints)
  bool has_bounds = false;
  for (int k : unfixed_indices)
  {
    if (k < lower_bounds.rows() && (std::isfinite(lower_bounds[k]) || std::isfinite(upper_bounds[k])))
    {
      has_bounds = true;
    }
  }
  const int ms = has_bounds ? n : 0;
  const int m = ms + rows;
  auto bound = [](double value) { return std::isinf(value) ? std::copysign((double)DAQP_INF, value) : value; };

  P.setZero(n, n);
  P.diagonal().setConstant(regularization);
  q.setZero(n);
  daqp_A.setZero(rows, n);
  daqp_lower.resize(m);
  daqp_upper.resize(m);
  daqp_lambda.setZero(m);
  daqp_rho.setZero(m);
  daqp_l1.setZero(m);
  daqp_sense.assign(m, 0);
  daqp_rows_mapping.assign(rows, nullptr);

  for (int k = 0; k < ms; k++)
  {
    int index = unfixed_indices[k];
    daqp_lower[k] = index < lower_bounds.rows() ? bound(lower_bounds[index]) : -DAQP_INF;
    daqp_upper[k] = index < upper_bounds.rows() ? bound(upper_bounds[index]) : DAQP_INF;
  }

  // Ax + b = 0 or Ax + b >= 0, as DAQP rows lower <= Ax <= upper
  int row = 0;
  for (int i = 0; i < (int)constraints.size(); i++)
  {
    ProblemConstraint* constraint = constraints[i];
    Reduced& r = reduced[i];
    const bool soft = constraint->priority == ProblemConstraint::Soft;
    const bool equality = constraint->type == ProblemConstraint::Equality;

    if (soft && equality)
    {
      compute_runs(r, runs, use_sparsity);
      add_squared_norm(r, runs, constraint->weight);
      continue;
    }

    const int count = r.A.rows();
    compute_runs(r, runs, false);
    for (const Run& run : runs)
    {
      daqp_A.block(row, run.var, count, run.size) = r.A.middleCols(run.col, run.size);
    }
    for (int k = 0; k < count; k++, row++)
    {
      // Hard rows are normalized; soft rows are not, since their weights are in the expression units
      const double norm = soft ? 1.0 : daqp_A.row(row).norm();
      const double scale = norm > 0 ? norm : 1.0;
      if (scale != 1.0)
      {
        daqp_A.row(row) /= scale;
      }
      const int j = ms + row;
      daqp_lower[j] = -r.b[k] / scale;
      daqp_upper[j] = equality ? daqp_lower[j] : DAQP_INF;
      daqp_rows_mapping[row] = constraint;
      if (soft)
      {
        daqp_sense[j] = DAQP_SOFT;
        daqp_rho[j] = 1.0 / constraint->weight;
        daqp_l1[j] = daqp_soft_l1_weight;
        slack_variables += 1;
        n_inequalities += 1;
      }
      else if (equality)
      {
        n_equalities += 1;
      }
      else
      {
        n_inequalities += 1;
      }
    }
  }

  qp_x.resize(n);
  if (n > 0)
  {
    // add_squared_norm fills the lower triangle, DAQP reads the full Hessian
    for (int j = 1; j < n; j++)
    {
      P.col(j).head(j) = P.row(j).head(j).transpose();
    }

    DAQPProblem qp = { n,
                       m,
                       ms,
                       P.data(),
                       q.data(),
                       rows ? daqp_A.data() : nullptr,
                       m ? daqp_upper.data() : nullptr,
                       m ? daqp_lower.data() : nullptr,
                       slack_variables ? daqp_sense.data() : nullptr,
                       nullptr,
                       0,
                       0 };
    DAQPResult result = {};
    result.x = qp_x.data();
    result.lam = m ? daqp_lambda.data() : nullptr;
    DAQPSettings settings;
    daqp_default_settings(&settings);

    if (slack_variables > 0)
    {
      // Per-row soft weights are set on the workspace, between the setup and the solve
      DAQPWorkspace work = {};
      work.settings = &settings;
      result.exitflag =
          setup_daqp_main(&qp, &work, &result.setup_time, DAQP_UPDATE_unconstrained | DAQP_UPDATE_eliminate);
      if (result.exitflag > 0)
      {
        bool weights_ok = daqp_set_soft_weights(&work, daqp_rho.data(), nullptr, daqp_l1.data(), nullptr);
        if (weights_ok)
        {
          daqp_solve(&result, &work);
        }
        work.settings = nullptr;
        free_daqp_workspace(&work);
        free_daqp_ldp(&work);
        if (!weights_ok)
        {
          throw QPError("Problem: DAQP rejected the soft constraint weights");
        }
      }
    }
    else
    {
      daqp_quadprog(&result, &qp, &settings);
    }

    if (result.exitflag != DAQP_EXIT_OPTIMAL && result.exitflag != DAQP_EXIT_SOFT_OPTIMAL)
    {
      throw QPError("Problem: DAQP failed to solve the QP (exitflag " + std::to_string(result.exitflag) + ")");
    }
  }

  x = fixed_values;
  for (int k = 0; k < n; k++)
  {
    x[unfixed_indices[k]] = qp_x[k];
  }
  if (!x.allFinite())
  {
    throw QPError("Problem: DAQP returned a non-finite solution");
  }

  for (int k = 0; k < ms; k++)
  {
    if (qp_x[k] < daqp_lower[k] - 1e-6 * (1 + std::abs(daqp_lower[k])) ||
        qp_x[k] > daqp_upper[k] + 1e-6 * (1 + std::abs(daqp_upper[k])))
    {
      throw QPError("Problem: DAQP solution violates the bounds");
    }
  }

  // Checking the hard rows, retrieving the active constraints and the slacks (violations of the soft inequalities)
  slacks.resize(slack_variables);
  int k_slack = 0;
  for (int k = 0; k < rows; k++)
  {
    const int j = ms + k;
    const double value = daqp_A.row(k).dot(qp_x) - daqp_lower[j];
    if (daqp_sense[j] == DAQP_SOFT)
    {
      slacks[k_slack++] = std::max(0.0, value);
      if (value <= 1e-6)
      {
        daqp_rows_mapping[k]->is_active = true;
      }
    }
    else
    {
      if (value < -1e-6 * (1 + std::abs(daqp_lower[j])) ||
          value > daqp_upper[j] - daqp_lower[j] + 1e-6 * (1 + std::abs(daqp_upper[j])))
      {
        throw QPError("Problem: DAQP solution violates a hard constraint");
      }
      if (daqp_lower[j] != daqp_upper[j] && daqp_lambda[j] != 0)
      {
        daqp_rows_mapping[k]->is_active = true;
      }
    }
  }

  for (auto variable : variables)
  {
    variable->version += 1;
    variable->value = x.segment(variable->k_start, variable->size());
  }
}
}  // namespace placo::problem
