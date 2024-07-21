#pragma once

#include <Eigen/Dense>
#include "placo/problem/sparsity.h"

namespace placo::problem
{
class ProblemConstraint;

/**
 * @brief An expression is a linear combination of decision variables of the form Ax + b that can
 * be conveniently manipulated using operators
 */
class Expression
{
public:
  Expression();
  Expression(const Expression& other);
  Expression(const Eigen::VectorXd& v);

  /**
   * @brief Expression A matrix, in Ax + b
   */
  Eigen::MatrixXd A = Eigen::MatrixXd(0, 0);

  /**
   * @brief Expression b vector, in Ax + b
   */
  Eigen::VectorXd b = Eigen::VectorXd(0);

  /**
   * @brief Slice rows from a given expression
   * @param start start row
   * @param rows number of rows (default: -1, all rows)
   * @return a sliced expression
   */
  Expression slice(int start, int rows = -1) const;

  /**
   * @brief Builds an expression from a vector (A will be zeros)
   * @param v vector
   * @return expression
   */
  static Expression from_vector(const Eigen::VectorXd& v);

  /**
   * @brief Builds an expression from a double (A will be zero, the expression is only one row)
   * @param value value
   * @return expression
   */
  static Expression from_double(const double& value);

  /**
   * @brief Retrieve the expression value, given a decision variable. This can be used after a problem is
   * solved to retrieve a specific expression value.
   *
   * For exemple, passing ``problem.x`` after calling \ref Problem::solve will return the value of the expression
   * ```cpp
   * // Value of the expression, given the problem solution
   * expr.value(problem.x);
   * ```
   *
   * @param x
   * @return
   */
  Eigen::VectorXd value(Eigen::VectorXd x) const;

  /**
   * @brief checks if the expression is a scalar
   * @return true if the expression is a scalar
   */
  bool is_scalar() const;

  /**
   * @brief checks if the expression is constant (doesn't depend on decision variables)
   * @return true if the expression is constant
   */
  bool is_constant() const;

  /**
   * @brief Number of cols in A
   * @return number of cols in A
   */
  int cols() const;

  /**
   * @brief Number of rows in A
   * @return number of rows in A
   */
  int rows() const;

  /**
   * @brief Adds the expression element by element to another expression
   * @param f
   * @return
   */
  Expression piecewise_add(double f) const;

  /**
   * @brief Multiply an expression on the left by a given matrix M
   * @param M matrix
   * @return expression
   */
  Expression left_multiply(const Eigen::MatrixXd M);

  /**
   * @brief Reduces a multi-rows expression to the sum of its items
   * @return expression
   */
  Expression sum();

  /**
   * @brief Reduces a multi-rows expression to the mean of its items
   * @return expression
   */
  Expression mean();

  // Summing expressions
  Expression operator+(const Expression& other) const;
  Expression operator-(const Expression& other) const;
  Expression operator-() const;

  // Multiplying by a scalar
  Expression operator*(double f) const;
  friend Expression operator*(double f, const Expression& e);
  Expression operator*(const Expression& other) const;

  // Adding a vector
  Expression operator+(const Eigen::VectorXd v) const;
  friend Expression operator+(const Eigen::VectorXd v, const Expression& e);
  Expression operator-(const Eigen::VectorXd v) const;
  friend Expression operator-(const Eigen::VectorXd v, const Expression& e);

  // Adding a number
  Expression operator+(const double f) const;
  friend Expression operator+(double, const Expression& e);
  Expression operator-(const double f) const;
  friend Expression operator-(double, const Expression& e);

  // Multiplying by a matrix
  friend Expression operator*(const Eigen::MatrixXd M, const Expression& e);

  // Stacking expressions
  Expression operator/(const Expression& other) const;

  // Comparing to produce constraints
  ProblemConstraint operator>=(const Expression& other) const;
  ProblemConstraint operator<=(const Expression& other) const;

  ProblemConstraint operator>=(double f) const;
  friend ProblemConstraint operator>=(double f, const Expression& e);

  ProblemConstraint operator<=(double f) const;
  friend ProblemConstraint operator<=(double f, const Expression& e);

  ProblemConstraint operator>=(Eigen::VectorXd v) const;
  friend ProblemConstraint operator>=(Eigen::VectorXd v, const Expression& e);

  ProblemConstraint operator<=(Eigen::VectorXd v) const;
  friend ProblemConstraint operator<=(Eigen::VectorXd v, const Expression& e);

  ProblemConstraint operator==(const Expression& other) const;
  ProblemConstraint operator==(Eigen::VectorXd v) const;

  friend ProblemConstraint operator==(Eigen::VectorXd v, const Expression& e);

  ProblemConstraint operator==(double f) const;
  friend ProblemConstraint operator==(double f, const Expression& e);
};
}  // namespace placo::problem