#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/kinematics/constraint.h"

namespace placo::kinematics
{
class KinematicsSolver;
class CoMPolygonConstraint : public Constraint
{
public:
  /**
   * @brief Ensures that the CoM (2D) lies inside the given polygon
   * @param polygon Clockwise polygon
   */
  CoMPolygonConstraint(const std::vector<Eigen::Vector2d>& polygon, double margin = 0.);

  /**
   * @brief Clockwise polygon
   */
  std::vector<Eigen::Vector2d> polygon;

  /**
   * @brief Margin for the polygon constraint (minimum distance between the CoM and the polygon)
   */
  double margin;

  virtual void add_constraint(placo::problem::Problem& problem) override;
};
}  // namespace placo::kinematics