#include <iostream>
#include <algorithm>
#include "placo/problem/expression.h"

namespace placo
{
Expression::Expression()
{
}

Expression::Expression(const Expression& other)
{
  A = other.A;
  b = other.b;
}

int Expression::cols() const
{
  return A.cols();
}

int Expression::rows() const
{
  return A.rows();
}

Expression Expression::operator+(const Expression& other)
{
  if (rows() != other.rows())
  {
    throw std::runtime_error("Trying to add expressions of different # rows");
  }

  Expression e;

  e.A = Eigen::MatrixXd(rows(), std::max(cols(), other.cols()));
  e.A.setZero();
  e.b = Eigen::VectorXd(rows());
  e.b.setZero();

  e.A.block(0, 0, rows(), cols()) = A.block(0, 0, rows(), cols());
  e.A.block(0, 0, rows(), other.cols()) += other.A.block(0, 0, rows(), other.cols());
  e.b = b;
  e.b += other.b;

  return e;
}

Expression Expression::operator-(const Expression& other)
{
  return (*this) + (other * (-1.));
}

Expression operator*(double f, Expression& e)
{
  return e * f;
}

Expression Expression::operator*(double f) const
{
  Expression e(*this);
  e.A *= f;
  e.b *= f;

  return e;
}

Expression Expression::operator+(const Eigen::MatrixXd v)
{
  Expression e(*this);
  e.b += v;
  return e;
}

Expression operator+(const Eigen::MatrixXd v, Expression& e)
{
  return e + v;
}

Expression Expression::operator-(const Eigen::MatrixXd v)
{
  Expression e(*this);
  e.b -= v;
  return e;
}

Expression operator-(const Eigen::MatrixXd v, Expression& e)
{
  return e - v;
}

Expression operator*(const Eigen::MatrixXd M, Expression& e_)
{
  Expression e(e_);
  e.A = M * e.A;
  e.b = M * e.b;

  return e;
}

Expression Expression::multiply(const Eigen::MatrixXd M)
{
  return M * (*this);
}

Expression Expression::sum()
{
  Expression e;
  e.A = Eigen::MatrixXd(1, cols());
  e.A.setZero();
  e.b = Eigen::VectorXd(1);
  e.b.setZero();

  for (int k = 0; k < rows(); k++)
  {
    e.A.block(0, 0, 1, cols()) += A.block(k, 0, 1, cols());
    e.b.block(0, 0, 1, 1) += b.block(k, 0, 1, 1);
  }

  return e;
}

Expression Expression::mean()
{
  Expression e = sum();
  e.A /= (double)cols();
  e.b /= (double)cols();

  return e;
}

Expression Expression::operator<<(const Expression& other)
{
  Expression e;

  e.A = Eigen::MatrixXd(rows() + other.rows(), std::max(cols(), other.cols()));
  e.A.setZero();
  e.b = Eigen::VectorXd(rows() + other.rows());
  e.b.setZero();

  e.A.block(0, 0, rows(), cols()) = A;
  e.A.block(rows(), 0, other.rows(), other.cols()) = other.A;

  e.b.block(0, 0, rows(), 1) = b;
  e.b.block(rows(), 0, other.rows(), 1) = other.b;

  return e;
}

};  // namespace placo