#include <iostream>
#include "placo/problem/polygon_constraint.h"

namespace placo::problem
{
std::vector<ProblemConstraint> PolygonConstraint::in_polygon_xy(const Expression& expression_xy,
                                                                std::vector<Eigen::Vector2d> polygon, double margin)
{
  std::vector<ProblemConstraint> constraints;

  if (expression_xy.rows() != 2)
  {
    throw std::runtime_error("add_polygon_constraint should be called with a 2 rows expressions");
  }

  for (size_t i = 0; i < polygon.size(); i++)
  {
    int j = (i + 1) % polygon.size();

    // Getting two consecutive points
    const Eigen::Vector2d& A = polygon[i];
    const Eigen::Vector2d& B = polygon[j];

    // Computing the (normalized) vector pointing inside the polygon (since it is clockwise)
    Eigen::Vector2d n;
    n << (B - A).y(), (A - B).x();
    n.normalize();

    // The distance to the line is given by n.T * (P - A) >= margin
    constraints.push_back((n.transpose() * (expression_xy - A)) >= margin);
  }

  return constraints;
}

std::vector<ProblemConstraint> PolygonConstraint::in_polygon(const Expression& expression_x,
                                                             const Expression& expression_y,
                                                             std::vector<Eigen::Vector2d> polygon, double margin)
{
  Expression e = expression_x / expression_y;
  return in_polygon_xy(e, polygon, margin);
}
};  // namespace placo::problem