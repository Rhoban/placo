#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/problem/constraints.h"

namespace placo
{
class PolygonConstraint
{
public:
  /**
   * @brief Given a polygon, produces inequalities so that the given point lies inside the polygon.
   * WARNING: Polygon must be clockwise (meaning that the exterior of the shape is on the trigonometric normal of
   * the vertices)
   *
   *             B
   *             X
   *           . .
   *         .   . ---> normal
   *       .     .
   *     X.......X
   *     A       C
   */
  static std::vector<ProblemConstraint> in_polygon_xy(const Expression& expression_xy,
                                                      std::vector<Eigen::Vector2d> polygon, double margin = 0.);

  static std::vector<ProblemConstraint> in_polygon(const Expression& expression_x, const Expression& expression_y,
                                                   std::vector<Eigen::Vector2d> polygon, double margin = 0.);
};
}  // namespace placo