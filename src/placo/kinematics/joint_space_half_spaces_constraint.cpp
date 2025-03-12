#include "placo/kinematics/joint_space_half_spaces_constraint.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/problem/polygon_constraint.h"

namespace placo::kinematics
{
JointSpaceHalfSpacesConstraint::JointSpaceHalfSpacesConstraint(const Eigen::MatrixXd A, Eigen::VectorXd b) : A(A), b(b)
{
}

void JointSpaceHalfSpacesConstraint::add_constraint(placo::problem::Problem& problem)
{
  if (A.rows() != b.rows())
  {
    throw std::runtime_error("Matrix A and b should have name number of rows in joint-space half-spaces constraint");
  }
  if (A.cols() != solver->N)
  {
    throw std::runtime_error("Matrix A should have nv cols in joint-space half-spaces constraint");
  }

  // We want Aq <= b
  // So A(q0 + dq) <= b
  placo::problem::Expression expression;
  expression.A = A;
  expression.b = A * solver->robot.state.q;

  problem.add_constraint(expression <= b)
      .configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                           problem::ProblemConstraint::Soft,
                 weight);
  ;
}

}  // namespace placo::kinematics