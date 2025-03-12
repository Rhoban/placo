#pragma once

#include <vector>
#include "placo/problem/problem.h"
#include "placo/kinematics/constraint.h"

namespace placo::kinematics
{
class KinematicsSolver;
class JointSpaceHalfSpacesConstraint : public Constraint
{
public:
  /**
   * @brief Ensures that, in joint-space we have Aq <= b
   *        Note that the floating base terms will be ignored in A. However, A should still be of
   *        dimension n_constraints x n_q
   */
  JointSpaceHalfSpacesConstraint(const Eigen::MatrixXd A, Eigen::VectorXd b);

  /**
   * @brief Matrix A in Aq <= b
   */
  Eigen::MatrixXd A;

  /**
   * @brief Vector b in Aq <= b
   */
  Eigen::VectorXd b;

  virtual void add_constraint(placo::problem::Problem& problem) override;
};
}  // namespace placo::kinematics