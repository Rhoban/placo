#include <iostream>
#include "placo/problem/polygon_constraint.h"

namespace placo
{
ProblemConstraints PolygonConstraint::add_polygon_constraint(Problem& problem, Expression& expression_x,
                                                             Expression& expression_y,
                                                             std::vector<Eigen::Vector2d> polygon, double margin)
{
  ProblemConstraints constraints;

  if (expression_x.rows() != 1 || expression_y.rows() != 1)
  {
    throw std::runtime_error("add_polygon_constraint should be called with 1 row expressions (for x and y)");
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

    // The distance to the line is given by n.T * (P - A) = n.x * (P.x - A.x) + n.Y * (P.y - A.y)
    ProblemConstraint& constraint =
        problem.add_constraint((n.x() * (expression_x - A.x()) + n.y() * (expression_y - A.y())) >= margin);
    constraints.constraints.push_back(&constraint);
  }

  return constraints;
}
};  // namespace placo