#include <algorithm>
#include <set>
#include "placo/problem/sparse_elimination.h"
#include "placo/problem/qp_error.h"

namespace placo::problem
{
void SparseElimination::analyze(int n_variables, const std::vector<std::vector<int>>& supports)
{
  // Graph of variables appearing together in an equality
  std::vector<std::set<int>> neighbors(n_variables);
  std::vector<bool> remaining(n_variables, false);
  for (auto& support : supports)
  {
    for (int i : support)
    {
      remaining[i] = true;
      neighbors[i].insert(support.begin(), support.end());
      neighbors[i].erase(i);
    }
  }

  // Minimum degree ordering: the variable with fewest neighbors is eliminated first, together with the variables
  // that are indistinguishable from it (same neighbors). Its neighbors (separator) then become a clique.
  steps.clear();
  while (true)
  {
    int best = -1;
    for (int v = 0; v < n_variables; v++)
    {
      if (remaining[v] && (best < 0 || neighbors[v].size() < neighbors[best].size()))
      {
        best = v;
      }
    }
    if (best < 0)
    {
      break;
    }

    std::set<int> closed_best = neighbors[best];
    closed_best.insert(best);

    Step step;
    step.block.push_back(best);
    for (int u : neighbors[best])
    {
      std::set<int> closed_u = neighbors[u];
      closed_u.insert(u);
      if (closed_u == closed_best)
      {
        step.block.push_back(u);
      }
      else
      {
        step.separator.push_back(u);
      }
    }
    std::sort(step.block.begin(), step.block.end());

    for (int v : step.block)
    {
      remaining[v] = false;
      for (int u : neighbors[v])
      {
        neighbors[u].erase(v);
      }
      neighbors[v].clear();
    }
    for (int a : step.separator)
    {
      neighbors[a].insert(step.separator.begin(), step.separator.end());
      neighbors[a].erase(a);
    }
    steps.push_back(step);
  }

  // Merging small blocks into the step that eliminates their separator (fewer, larger dense factorizations). This
  // keeps the elimination valid: the separator of a step is always contained in the block and separator of that
  // later step.
  std::vector<int> step_of(n_variables, -1);
  for (int i = 0; i < (int)steps.size(); i++)
  {
    for (int v : steps[i].block)
    {
      step_of[v] = i;
    }
  }
  std::vector<bool> merged(steps.size(), false);
  for (int i = 0; i < (int)steps.size(); i++)
  {
    if (steps[i].separator.empty())
    {
      continue;
    }
    int parent = steps.size();
    for (int v : steps[i].separator)
    {
      parent = std::min(parent, step_of[v]);
    }
    Step& p = steps[parent];
    if ((int)(steps[i].block.size() + p.block.size()) > max_block_size)
    {
      continue;
    }
    p.block.insert(p.block.end(), steps[i].block.begin(), steps[i].block.end());
    std::sort(p.block.begin(), p.block.end());
    std::set<int> separator(p.separator.begin(), p.separator.end());
    for (int v : steps[i].separator)
    {
      if (!std::binary_search(p.block.begin(), p.block.end(), v))
      {
        separator.insert(v);
      }
    }
    p.separator.assign(separator.begin(), separator.end());
    for (int v : steps[i].block)
    {
      step_of[v] = parent;
    }
    merged[i] = true;
  }
  std::vector<Step> kept;
  for (int i = 0; i < (int)steps.size(); i++)
  {
    if (!merged[i])
    {
      kept.push_back(steps[i]);
    }
  }
  steps = kept;

  // Equalities involved in each step: the original equalities touching its block that were not used yet, and the rows
  // left by previous steps on their separator (generated factor i has index n_equalities + i)
  int n_equalities = supports.size();
  std::vector<std::vector<int>> factor_variables = supports;
  factor_variables.resize(n_equalities + steps.size());
  std::vector<bool> used(n_equalities + steps.size(), false);
  std::vector<std::vector<int>> factors_of(n_variables);
  for (int f = 0; f < n_equalities; f++)
  {
    for (int v : supports[f])
    {
      factors_of[v].push_back(f);
    }
  }
  for (int i = 0; i < (int)steps.size(); i++)
  {
    for (int v : steps[i].block)
    {
      for (int f : factors_of[v])
      {
        if (!used[f])
        {
          used[f] = true;
          steps[i].factors.push_back(f);
        }
      }
    }
    int generated = n_equalities + i;
    factor_variables[generated] = steps[i].separator;
    for (int v : steps[i].separator)
    {
      factors_of[v].push_back(generated);
    }
  }
}

int SparseElimination::steps_count() const
{
  return steps.size();
}

bool SparseElimination::eliminate(int n_variables, const std::vector<Equality>& equalities)
{
  // Structure of the equalities (the analysis is only done again when it changes)
  std::vector<int> structure = { n_variables };
  for (auto& equality : equalities)
  {
    structure.push_back(equality.A->rows());
    structure.push_back(equality.columns->size());
    structure.insert(structure.end(), equality.columns->begin(), equality.columns->end());
  }
  if (structure != analyzed_structure)
  {
    supports.clear();
    for (auto& equality : equalities)
    {
      supports.push_back(*equality.columns);
    }
    analyze(n_variables, supports);
    analyzed_structure = structure;
  }

  // Without structure (all the constrained variables in one block), a dense QR elimination is preferred
  if (steps.size() <= 1)
  {
    status = "no_structure";
    return false;
  }

  // Rows A x_separator + b = 0 generated by each step on its separator, passed to the next steps (factors of index
  // n_equalities + s)
  struct Generated
  {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
  };
  int n_equalities = equalities.size();
  std::vector<Generated> generated_rows(steps.size());
  auto factor_rows = [&](int f) {
    return f < n_equalities ? equalities[f].A->rows() : generated_rows[f - n_equalities].A.rows();
  };

  // Conditional produced by each step: x_pivots = K_free x_block_free + K_separator x_separator + k0
  struct Conditional
  {
    std::vector<int> pivots, block_free;
    Eigen::MatrixXd K_free, K_separator;
    Eigen::VectorXd k0;
  };
  std::vector<Conditional> conditionals(steps.size());
  std::vector<int> local(n_variables, -1);

  for (int s = 0; s < (int)steps.size(); s++)
  {
    const Step& step = steps[s];
    Conditional& conditional = conditionals[s];
    int nb = step.block.size(), ns = step.separator.size();

    // Gathering the rows involved in this step: M = [A_block | A_separator], rhs
    for (int i = 0; i < nb; i++)
    {
      local[step.block[i]] = i;
    }
    for (int i = 0; i < ns; i++)
    {
      local[step.separator[i]] = nb + i;
    }
    int rows = 0;
    for (int f : step.factors)
    {
      rows += factor_rows(f);
    }
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(rows, nb + ns);
    Eigen::VectorXd rhs(rows);
    int row = 0;
    for (int f : step.factors)
    {
      int f_rows = factor_rows(f);
      if (f < n_equalities)
      {
        // Original equality (the k-th column of A is the variable supports[f][k])
        for (int k = 0; k < (int)supports[f].size(); k++)
        {
          M.block(row, local[supports[f][k]], f_rows, 1) = equalities[f].A->col(k);
        }
        rhs.segment(row, f_rows) = *equalities[f].b;

        // Its rows are normalized, so that the conditioning checks don't depend on the scale of each equality. Rows
        // generated by previous steps are not: they are numerically zero when equalities are redundant, and should
        // stay so to be detected by the rank-revealing factorization.
        for (int r = row; r < row + f_rows; r++)
        {
          double norm = M.row(r).norm();
          if (norm > 0)
          {
            M.row(r) /= norm;
            rhs[r] /= norm;
          }
        }
      }
      else
      {
        // Rows generated by a previous step, on its separator
        const Generated& g = generated_rows[f - n_equalities];
        const std::vector<int>& variables = steps[f - n_equalities].separator;
        for (int j = 0; j < (int)variables.size(); j++)
        {
          M.block(row, local[variables[j]], f_rows, 1) = g.A.col(j);
        }
        rhs.segment(row, f_rows) = g.b;
      }
      row += f_rows;
    }
    for (int v : step.block)
    {
      local[v] = -1;
    }
    for (int v : step.separator)
    {
      local[v] = -1;
    }

    Generated& generated = generated_rows[s];
    generated.A.resize(0, ns);
    generated.b.resize(0);

    if (rows == 0)
    {
      conditional.block_free = step.block;
      continue;
    }

    // Square blocks: LU with partial pivoting when it is well conditioned
    if (rows == nb)
    {
      Eigen::PartialPivLU<Eigen::MatrixXd> lu(M.leftCols(nb));
      Eigen::VectorXd pivots_magnitude = lu.matrixLU().diagonal().cwiseAbs();
      if (pivots_magnitude.minCoeff() > min_pivot_ratio * pivots_magnitude.maxCoeff())
      {
        conditional.pivots = step.block;
        conditional.K_free.resize(nb, 0);
        conditional.K_separator = -lu.solve(M.rightCols(ns));
        conditional.k0 = -lu.solve(rhs);
        continue;
      }
    }

    // General case: rank-revealing QR of the block columns, A_block P = Q [R11 R12; 0 0]
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(M.leftCols(nb));
    int rank = qr.rank();
    if (rank > 0)
    {
      Eigen::VectorXd diagonal = qr.matrixR().diagonal().head(rank).cwiseAbs();
      if (diagonal.minCoeff() < min_pivot_ratio * diagonal.maxCoeff())
      {
        // Poorly conditioned block: expressing its pivots from its other variables would amplify rounding errors
        status = "poorly_conditioned_block";
        return false;
      }
    }
    Eigen::MatrixXd separator_part = qr.householderQ().adjoint() * M.rightCols(ns);
    Eigen::VectorXd rhs_part = qr.householderQ().adjoint() * rhs;
    auto R11 = qr.matrixR().topLeftCorner(rank, rank).triangularView<Eigen::Upper>();

    const auto& permutation = qr.colsPermutation().indices();
    for (int i = 0; i < nb; i++)
    {
      (i < rank ? conditional.pivots : conditional.block_free).push_back(step.block[permutation[i]]);
    }
    conditional.K_free = -R11.solve(qr.matrixR().block(0, rank, rank, nb - rank));
    conditional.K_separator = -R11.solve(separator_part.topRows(rank));
    conditional.k0 = -R11.solve(rhs_part.head(rank));

    // Remaining rows only involve the separator: they are passed to the next steps
    if (rows > rank)
    {
      if (ns == 0)
      {
        throw QPError("Sparse elimination failed to find a full rank matrix for equality constraints");
      }
      generated.A = separator_part.bottomRows(rows - rank);
      generated.b = rhs_part.tail(rows - rank);
    }
  }

  // Back substitution: pivots expressed as a function of the free variables, last steps first
  pivots.clear();
  free.clear();
  std::vector<int> pivot_row(n_variables, -1), free_column(n_variables, -1);
  for (auto& conditional : conditionals)
  {
    for (int v : conditional.pivots)
    {
      pivot_row[v] = pivots.size();
      pivots.push_back(v);
    }
  }
  for (int v = 0; v < n_variables; v++)
  {
    if (pivot_row[v] < 0)
    {
      free_column[v] = free.size();
      free.push_back(v);
    }
  }

  Z = Eigen::MatrixXd::Zero(pivots.size(), free.size());
  x0 = Eigen::VectorXd::Zero(pivots.size());
  for (int s = steps.size() - 1; s >= 0; s--)
  {
    const Conditional& conditional = conditionals[s];
    const std::vector<int>& separator = steps[s].separator;
    for (int i = 0; i < (int)conditional.pivots.size(); i++)
    {
      int r = pivot_row[conditional.pivots[i]];
      x0[r] = conditional.k0[i];
      for (int j = 0; j < (int)conditional.block_free.size(); j++)
      {
        Z(r, free_column[conditional.block_free[j]]) += conditional.K_free(i, j);
      }
      for (int j = 0; j < (int)separator.size(); j++)
      {
        double k = conditional.K_separator(i, j);
        int v = separator[j];
        if (free_column[v] >= 0)
        {
          Z(r, free_column[v]) += k;
        }
        else
        {
          Z.row(r) += k * Z.row(pivot_row[v]);
          x0[r] += k * x0[pivot_row[v]];
        }
      }
    }
  }

  // A large Z means a poorly conditioned parametrization of the solutions (see max_Z_norm)
  Z_norm = Z.norm();
  if (Z_norm > max_Z_norm)
  {
    status = "large_Z";
    return false;
  }

  status = "eliminated";
  Z_columns.resize(pivots.size());
  for (int r = 0; r < (int)pivots.size(); r++)
  {
    Z_columns[r].clear();
    for (int c = 0; c < (int)free.size(); c++)
    {
      if (Z(r, c) != 0)
      {
        Z_columns[r].push_back(c);
      }
    }
  }

  return true;
}
}  // namespace placo::problem
