#include <sstream>
#include <iostream>
#include <algorithm>
#include "placo/problem/expression.h"
#include "placo/problem/constraint.h"

namespace placo::problem
{
Expression::Expression()
{
}

Expression Expression::from_vector(const Eigen::VectorXd& v)
{
  return Expression(v);
}

Expression Expression::from_double(const double& value)
{
  Expression e;
  e.A = Eigen::MatrixXd(1, 0);
  e.b = Eigen::VectorXd(1);
  e.b(0, 0) = value;

  return e;
}

Expression::Expression(const Expression& other)
{
  A = other.A;
  b = other.b;
}

Expression::Expression(const Eigen::VectorXd& v)
{
  A = Eigen::MatrixXd(v.rows(), 0);
  b = v;
}

bool Expression::is_scalar() const
{
  return rows() == 1;
}

bool Expression::is_constant() const
{
  return cols() == 0;
}

Expression Expression::slice(int start, int rows) const
{
  Expression e;

  if (rows < 0)
  {
    rows = this->rows() - start;
  }

  e.A = A.block(start, 0, rows, cols());
  e.b = b.block(start, 0, rows, 1);

  return e;
}

int Expression::cols() const
{
  return A.cols();
}

int Expression::rows() const
{
  return A.rows();
}

Expression Expression::piecewise_add(double f) const
{
  // Assuming a scalar piece-wise sum
  Expression e(*this);

  for (int k = 0; k < e.b.rows(); k++)
  {
    e.b(k, 0) += f;
  }

  return e;
}

Expression Expression::operator+(const Expression& other) const
{
  if (is_scalar() && is_constant())
  {
    return other.piecewise_add(b(0, 0));
  }
  else if (other.is_scalar() && other.is_constant())
  {
    return piecewise_add(other.b(0, 0));
  }

  if (rows() != other.rows())
  {
    std::ostringstream oss;
    oss << "Trying to add expressions with different # of rows (" << rows() << " vs " << other.rows() << ")";
    throw std::runtime_error(oss.str());
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

Expression Expression::operator-(const Expression& other) const
{
  return (*this) + (other * (-1.));
}

Expression Expression::operator-() const
{
  return (*this) * (-1.);
}

Expression operator*(double f, const Expression& e)
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

Expression Expression::operator*(const Expression& other) const
{
  if (is_scalar() && other.is_constant())
  {
    Expression e;
    e.A.resize(other.rows(), cols());
    e.b.resize(other.rows());
    for (int k = 0; k < other.rows(); k++)
    {
      e.A.row(k) = A.row(0) * other.b(k);
      e.b(k) = b(0) * other.b(k);
    }

    return e;
  }
  else if (other.is_scalar() && is_constant())
  {
    return other * (*this);
  }
  else
  {
    throw std::runtime_error("Two expression can only be multiplied if one is scalar and the other constant");
  }
}

Expression Expression::operator+(const Eigen::VectorXd v) const
{
  Expression e(*this);
  e.b += v;

  return e;
}

Expression operator+(const Eigen::VectorXd v, const Expression& e)
{
  return e + v;
}

Expression Expression::operator-(const Eigen::VectorXd v) const
{
  Expression e(*this);
  e.b -= v;

  return e;
}

Expression operator-(const Eigen::VectorXd v, const Expression& e)
{
  return e - v;
}

Expression Expression::operator+(const double f) const
{
  Eigen::VectorXd fv(1);
  fv(0, 0) = f;

  return (*this) + fv;
}

Expression operator+(double f, const Expression& e)
{
  return e + f;
}

Expression Expression::operator-(const double f) const
{
  Eigen::VectorXd fv(1);
  fv(0, 0) = f;

  return (*this) - fv;
}

Expression operator-(double f, const Expression& e)
{
  return (-e) + f;
}

Expression operator*(const Eigen::MatrixXd M, const Expression& e_)
{
  Expression e(e_);
  e.A = M.operator*(e.A);
  e.b = M.operator*(e.b);

  return e;
}

Expression Expression::left_multiply(const Eigen::MatrixXd M)
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

Expression Expression::operator/(const Expression& other) const
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

ProblemConstraint Expression::operator>=(const Expression& other) const
{
  ProblemConstraint constraint;

  constraint.expression = *this - other;
  constraint.type = ProblemConstraint::Inequality;

  return constraint;
}

ProblemConstraint Expression::operator<=(const Expression& other) const
{
  ProblemConstraint constraint;

  constraint.expression = -(*this - other);
  constraint.type = ProblemConstraint::Inequality;

  return constraint;
}

Eigen::VectorXd Expression::value(Eigen::VectorXd x) const
{
  return A * x.block(0, 0, A.cols(), 1) + b;
}

ProblemConstraint Expression::operator>=(double f) const
{
  return (*this) >= Expression::from_double(f);
}

ProblemConstraint operator>=(double f, const Expression& e)
{
  return Expression::from_double(f) >= e;
}

ProblemConstraint Expression::operator<=(double f) const
{
  return (*this) <= Expression::from_double(f);
}

ProblemConstraint operator<=(double f, const Expression& e)
{
  return Expression::from_double(f) <= e;
}

ProblemConstraint Expression::operator>=(Eigen::VectorXd v) const
{
  return (*this) >= Expression::from_vector(v);
}

ProblemConstraint operator>=(Eigen::VectorXd v, const Expression& e)
{
  return Expression::from_vector(v) >= e;
}

ProblemConstraint Expression::operator<=(Eigen::VectorXd v) const
{
  return (*this) <= Expression::from_vector(v);
}

ProblemConstraint operator<=(Eigen::VectorXd v, const Expression& e)
{
  return Expression::from_vector(v) <= e;
}

ProblemConstraint Expression::operator==(const Expression& other) const
{
  ProblemConstraint constraint;
  constraint.expression = (*this) - other;

  return constraint;
}

ProblemConstraint Expression::operator==(Eigen::VectorXd v) const
{
  return (*this) == Expression::from_vector(v);
}

ProblemConstraint operator==(Eigen::VectorXd v, const Expression& e)
{
  return e == v;
}

ProblemConstraint Expression::operator==(double f) const
{
  return (*this) == Expression::from_double(f);
}

ProblemConstraint operator==(double f, const Expression& e)
{
  return e == f;
}
};  // namespace placo::problem