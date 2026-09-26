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

  int ndof = solver->N - 6;
  auto A_no_fbase = A.rightCols(ndof);

  // We want Aq <= b
  // So A(q0 + dq) <= b, the constraint only depends on the joints (not the floating base)
  problem::ProblemConstraint& constraint = problem.add_constraint();
  constraint.type = problem::ProblemConstraint::Inequality;
  constraint.columns.resize(ndof);
  for (int k = 0; k < ndof; k++)
  {
    constraint.columns[k] = 6 + k;
  }
  constraint.expression.A = -A_no_fbase;
  constraint.expression.b = b - A_no_fbase * solver->robot.state.q.bottomRows(ndof);
  constraint.configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard : problem::ProblemConstraint::Soft, weight);
}

}  // namespace placo::kinematics