#include <iostream>
#include "placo/problem/polygon_constraint.h"

namespace placo
{
ProblemConstraints PolygonConstraint::add_polygon_constraint(Problem& problem, Expression& expression,
                                                             std::vector<Eigen::Vector2d> polygon, double margin)
{
  ProblemConstraints constraints;

  if (expression.rows() != 2)
  {
    throw std::runtime_error("add_polygon_constraint should be called with a 2 rows expression (x, y)");
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

    ProblemConstraint& constraint = problem.add_constraint((n.transpose() * (expression - A)) >= margin);
    constraints.constraints.push_back(&constraint);
  }

  return constraints;
}
};  // namespace placo