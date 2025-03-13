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
  if (A.cols() != solver->robot.state.q.rows())
  {
    throw std::runtime_error("Matrix A should have ndof cols in joint-space half-spaces constraint");
  }

  Eigen::MatrixXd A_no_fbase = A.block(0, 7, A.rows(), A.cols() - 7);
  int ndof = solver->N - 6;

  // We want Aq <= b
  // So A(q0 + dq) <= b
  placo::problem::Expression expression;
  expression.A = Eigen::MatrixXd(A.rows(), solver->N);
  expression.A.setZero();
  expression.A.block(0, 6, A_no_fbase.rows(), ndof) = A_no_fbase;

  expression.b = Eigen::VectorXd(A.rows());
  expression.b.setZero();
  expression.b = A_no_fbase * solver->robot.state.q.block(7, 0, ndof, 1);

  problem.add_constraint(expression <= b)
      .configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                           problem::ProblemConstraint::Soft,
                 weight);
  ;
}

}  // namespace placo::kinematics