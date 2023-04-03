#pragma once

#include <Eigen/Dense>

namespace placo
{
class Expression
{
public:
  Expression();
  Expression(const Expression& other);

  // An expression is Ax + b, where x is the decision variable
  Eigen::MatrixXd A = Eigen::MatrixXd(0, 0);
  Eigen::VectorXd b = Eigen::VectorXd(0);

  int cols() const;
  int rows() const;

  // Summing expressions
  Expression operator+(const Expression& other);
  Expression operator-(const Expression& other);

  // Multiplying by a scalar
  Expression operator*(double f) const;
  friend Expression operator*(double f, Expression& e);

  // Adding a vector
  Expression operator+(const Eigen::MatrixXd v);
  friend Expression operator+(const Eigen::MatrixXd v, Expression& e);
  Expression operator-(const Eigen::MatrixXd v);
  friend Expression operator-(const Eigen::MatrixXd v, Expression& e);

  // Multiplying by a matrix
  friend Expression operator*(const Eigen::MatrixXd M, Expression& e);
  Expression multiply(const Eigen::MatrixXd M);

  // Sum of all stacked expressions
  Expression sum();
  Expression mean();

  // Stacking expressions
  Expression operator<<(const Expression &other);
};
}  // namespace placo