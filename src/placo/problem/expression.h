#pragma once

#include <Eigen/Dense>
#include "placo/problem/sparsity.h"

namespace placo
{
class ProblemConstraint;
class Expression
{
public:
  Expression();
  Expression(const Expression& other);

  static Expression from_vector(const Eigen::VectorXd& v);
  static Expression from_double(const double& value);

  // An expression is Ax + b, where x is the decision variable
  Eigen::MatrixXd A = Eigen::MatrixXd(0, 0);
  Eigen::VectorXd b = Eigen::VectorXd(0);

  Expression slice(int start, int rows) const;

  bool is_scalar() const;

  int cols() const;
  int rows() const;

  Expression piecewise_add(double f) const;

  // Summing expressions
  Expression operator+(const Expression& other) const;
  Expression operator-(const Expression& other) const;
  Expression operator-() const;

  // Multiplying by a scalar
  Expression operator*(double f) const;
  friend Expression operator*(double f, const Expression& e);

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
  Expression multiply(const Eigen::MatrixXd M);

  // Sum of all stacked expressions
  Expression sum();
  Expression mean();

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
}  // namespace placo