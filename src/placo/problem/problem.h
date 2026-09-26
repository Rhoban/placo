#pragma once

#include <memory>
#include <string>
#include <vector>
#include "placo/problem/expression.h"
#include "placo/problem/variable.h"
#include "placo/problem/constraint.h"
#include "placo/problem/qp_error.h"
#include "placo/problem/sparse_elimination.h"
#include <qpmad/solver.h>

namespace placo::problem
{
/**
 * @brief A problem is an object that has variables and constraints to be solved by a QP solver.
 */
class Problem
{
public:
  Problem();
  virtual ~Problem();

  /**
   * @brief Adds a n-dimensional variable to a problem
   * @param size dimension of the variable
   * @return variable
   */
  Variable& add_variable(int size = 1);

  /**
   * @brief Adds a limit, "absolute" inequality constraint (abs(Ax + b) <= t)
   * @param expression
   * @param target
   * @return The constraint
   */
  ProblemConstraint& add_limit(Expression expression, Eigen::VectorXd target);

  /**
   * @brief Adds a given constraint to the problem
   * @param constraint
   * @return The constraint
   */
  ProblemConstraint& add_constraint(const ProblemConstraint& constraint);

  /**
   * @brief Adds a constraint to be filled in place by the caller (hard equality by default). This avoids building
   * an expression and copying it, and allows compact constraints (see ProblemConstraint::columns). The constraint
   * objects (and the memory of their matrices) are reused from a solve to another (see \ref clear_constraints).
   * @return The constraint
   */
  ProblemConstraint& add_constraint();

  /**
   * @brief Adds bounds lower <= x <= upper on some values x of a variable (variable[start], ..., variable[start + n -
   * 1]). This is equivalent to hard inequality constraints, but bounds are handled more efficiently, and bounds on
   * the same values are merged (the tightest are kept). Infinite values can be used for one-sided bounds. Values with
   * equal lower and upper bounds are fixed, and removed from the QP (see \ref fixed_variables). Bounds are removed
   * with the constraints (see \ref clear_constraints).
   * @param variable variable
   * @param start index of the first bounded value in the variable
   * @param lower lower bounds
   * @param upper upper bounds
   */
  void add_bounds(const Variable& variable, int start, const Eigen::Ref<const Eigen::VectorXd>& lower,
                  const Eigen::Ref<const Eigen::VectorXd>& upper);

  /**
   * @brief Clear all the constraints (and bounds). The constraint objects are kept to be reused by the next calls
   * to \ref add_constraint.
   */
  void clear_constraints();

  /**
   * @brief Clear all the variables
   */
  void clear_variables();

  /**
   * @brief Solves the problem, raises \ref QPError in case of failure
   */
  void solve();

  /**
   * @brief Number of problem variables that need to be solved
   */
  int n_variables = 0;

  /**
   * @brief Number of inequality constraints
   */
  int n_inequalities = 0;

  /**
   * @brief Number of equalities
   */
  int n_equalities = 0;

  /**
   * @brief Number of free variables to solve.
   *
   * If \ref rewrite_equalities is true, this should be equals to \ref n_variable.
   */
  int free_variables = 0;

  /**
   * @brief Number of fixed variables (values with equal lower and upper bounds, see \ref add_bounds). They are
   * substituted in the constraints, and are not variables of the QP.
   */
  int fixed_variables = 0;

  /**
   * @brief Number of slack variables in the solver.
   */
  int slack_variables = 0;

  /**
   * @brief Number of determined variables
   *
   * If \ref rewrite_equalities is true, this should be equals to 0.
   */
  int determined_variables = 0;

  /**
   * @brief Default internal regularization
   */
  double regularization = 1e-8;

  /**
   * @brief Computed result
   */
  Eigen::VectorXd x;

  /**
   * @brief Computed slack variables
   */
  Eigen::VectorXd slacks;

  /**
   * @brief If set to true, the columns of dense constraints that are entirely zero are detected, and skipped when
   * building the problem Hessian. Compact constraints (see ProblemConstraint::columns) don't need this detection.
   */
  bool use_sparsity = true;

  /**
   * @brief If set to true, the hard equality constraints are eliminated before calling the QP solver (with a QR
   * factorization, or with a sparse elimination if \ref sparse_elimination is set), and the QP is called with free
   * variables only. Else, they are passed to the QP solver.
   *
   * The number of free variables will be available in \ref free_variables, and the number of determined variables
   * in \ref determined_variables.
   */
  bool rewrite_equalities = true;

  /**
   * @brief If set to true (and \ref rewrite_equalities is set), the hard equality constraints are eliminated
   * exploiting their structure (the variables each of them depends on, see SparseElimination) instead of a dense QR
   * factorization. This is useful when equalities are independent of each other (for instance loop closures on
   * different parts of a robot). It falls back to the QR factorization when the equalities have no such structure.
   */
  bool sparse_elimination = false;

  /**
   * @brief True if the sparse elimination was used for the last solve (see \ref sparse_elimination)
   */
  bool sparse_elimination_used = false;

  void dump_status();

protected:
  /**
   * @brief Bounds on the problem variables (infinite when there is no bound), see \ref add_bounds
   */
  Eigen::VectorXd lower_bounds, upper_bounds;

  /**
   * @brief Indices of the variables that are not fixed, index of each variable among them (-1 for fixed variables),
   * and values of the fixed variables (zero for the others), see \ref fixed_variables
   */
  std::vector<int> unfixed_indices;
  std::vector<int> unfixed_index;
  Eigen::VectorXd fixed_values;

  /**
   * @brief Updates \ref fixed_variables, \ref unfixed_indices, \ref unfixed_index and \ref fixed_values from the
   * bounds
   */
  void detect_fixed_variables();

  /**
   * @brief Number of finite bounds (lower and upper), counted as inequalities in \ref n_inequalities
   */
  int bounds_inequalities() const;

  /**
   * @brief QP solver
   */
  qpmad::Solver qp_solver;

  /**
   * @brief Internal object to store the QR decomposition
   */
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, -1, -1, 1, -1, -1>> QR;

  /**
   * @brief Internal vector of determined values (in the Q basis)
   */
  Eigen::MatrixXd y;

  /**
   * @brief Sparse elimination of the equalities, see \ref sparse_elimination
   */
  SparseElimination elimination;

  /**
   * @brief Index of each unfixed variable among the free variables of the sparse elimination (-1 if eliminated), and
   * row of the eliminated ones in SparseElimination::Z
   */
  std::vector<int> free_column, eliminated_row;

  /**
   * @brief Problem variables
   */
  std::vector<Variable*> variables;

  /**
   * @brief Problem constraints
   */
  std::vector<ProblemConstraint*> constraints;

  /**
   * @brief Constraint objects that were cleared, reused by \ref add_constraint
   */
  std::vector<ProblemConstraint*> constraints_pool;

  /**
   * @brief A constraint expression A x[columns] + b after the substitution of the fixed variables (in the space of the
   * unfixed variables), and then after the elimination of the equalities (in the space of the QP variables)
   */
  struct Reduced
  {
    Eigen::Map<const Eigen::MatrixXd> A{ nullptr, 0, 0 };

    /**
     * @brief Variables of the columns of A (nullptr if A is dense: its k-th column is the variable k)
     */
    const std::vector<int>* columns = nullptr;

    Eigen::VectorXd b;

    // Storages for A and columns, when they are not the ones of the constraint
    Eigen::MatrixXd A_fixed, A_eliminated;
    std::vector<int> columns_fixed, columns_eliminated;
  };

  /**
   * @brief Reduced expressions, in the order of the constraints (kept from a solve to another to reuse the memory)
   */
  std::vector<Reduced> reduced;

  /**
   * @brief Substitutes the fixed variables in the constraint expression
   */
  void reduce_fixed(const ProblemConstraint& constraint, Reduced& r);

  /**
   * @brief Expresses the (reduced) expression as a function of the QP variables, according to the elimination of the
   * equalities
   */
  void reduce_eliminated(Reduced& r);

  /**
   * @brief Consecutive variables var, ..., var + size - 1 in consecutive columns col, ..., col + size - 1 of a matrix
   */
  struct Run
  {
    int var;
    int col;
    int size;
  };

  /**
   * @brief Computes the runs of a reduced expression. For dense expressions, zero columns are skipped if
   * \ref use_sparsity is set
   */
  void compute_runs(const Reduced& r, std::vector<Run>& runs, bool detect_zeros);

  /**
   * @brief Adds weight * ||A x + b||^2 to the objective (lower triangle of P)
   */
  void add_squared_norm(const Reduced& r, const std::vector<Run>& runs, double weight);

  // Workspaces (kept from a solve to another to reuse the memory)
  Eigen::MatrixXd P, C, Aeq, gram, bounds_full, R, b2;
  Eigen::VectorXd q, lower, upper, lb, ub, qp_x, beq, unfixed_x;
  std::vector<Run> runs;
  std::vector<int> stamp, gathered;
  std::vector<ProblemConstraint*> hard_inequalities_mapping, soft_inequalities_mapping;
};
}  // namespace placo::problem
