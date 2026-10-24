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

  // We want Aq <= b, A having a column per configuration coordinate
  // So A(q0 + dq) <= b, the constraint only depends on the joints (not the floating base), through the degrees of
  // freedom that have a configuration coordinate of their own (see RobotWrapper::v_to_q)
  int n_joints_q = A.cols() - 7;
  problem::ProblemConstraint& constraint = problem.add_constraint();
  constraint.type = problem::ProblemConstraint::Inequality;
  constraint.columns.clear();
  for (int k = 6; k < solver->N; k++)
  {
    if (solver->robot.v_to_q[k] >= 0)
    {
      constraint.columns.push_back(k);
    }
  }
  constraint.expression.A.resize(A.rows(), constraint.columns.size());
  for (int k = 0; k < (int)constraint.columns.size(); k++)
  {
    constraint.expression.A.col(k) = -A.col(solver->robot.v_to_q[constraint.columns[k]]);
  }
  constraint.expression.b = b - A.rightCols(n_joints_q) * solver->robot.state.q.bottomRows(n_joints_q);
  constraint.configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                                 problem::ProblemConstraint::Soft,
                       weight);
}

}  // namespace placo::kinematics