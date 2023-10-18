#include "placo/dynamics/contacts.h"
#include "placo/dynamics/dynamics_solver.h"
#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"
#include "placo/dynamics/frame_task.h"

// Some helpers for readability
#define F_X 0
#define F_Y 1
#define F_Z 2
#define M_X 3
#define M_Y 4
#define M_Z 5

namespace placo::dynamics
{
Contact::Contact()
{
}

Eigen::Vector3d PlanarContact::zmp()
{
  Eigen::Vector3d wrench = variable->value;
  return Eigen::Vector3d(-wrench(M_Y, 0) / wrench(F_Z, 0), wrench(M_X, 0) / wrench(F_Z, 0), 0);
}

PointContact::PointContact(PositionTask& position_task, bool unilateral)
{
  this->position_task = &position_task;
  this->unilateral = unilateral;
}

Contact::Wrench PointContact::add_wrench(Problem& problem)
{
  variable = &problem.add_variable(3);

  if (unilateral)
  {
    // The contact is unilateral
    problem.add_constraint(variable->expr(F_Z, 1) >= 0);

    // We don't slip
    problem.add_constraint(variable->expr(F_X, 1) <= mu * variable->expr(F_Z, 1));
    problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_X, 1));

    problem.add_constraint(variable->expr(F_Y, 1) <= mu * variable->expr(F_Z, 1));
    problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_Y, 1));
  }

  // Objective
  if (weight_forces > 0)
  {
    problem.add_constraint(variable->expr(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);
  }

  Contact::Wrench wrench;
  wrench.J = position_task->A;
  wrench.f = variable->expr();

  return wrench;
}

RelativePointContact::RelativePointContact(RelativePositionTask& relative_position_task)
{
  this->relative_position_task = &relative_position_task;
}

Contact::Wrench RelativePointContact::add_wrench(Problem& problem)
{
  variable = &problem.add_variable(relative_position_task->A.rows());

  // Objective
  if (weight_forces > 0)
  {
    problem.add_constraint(variable->expr() == 0).configure(ProblemConstraint::Soft, weight_forces);
  }

  Contact::Wrench wrench;
  wrench.J = relative_position_task->A;
  wrench.f = variable->expr();

  return wrench;
}

PlanarContact::PlanarContact(FrameTask& frame_task, bool unilateral)
{
  this->position_task = frame_task.position;
  this->orientation_task = frame_task.orientation;
  this->unilateral = unilateral;
}

Contact::Wrench PlanarContact::add_wrench(Problem& problem)
{
  // Wrench is: [ fx fy fz mx my mz ] with f the force and m the moment
  variable = &problem.add_variable(6);

  if (unilateral)
  {
    // The contact is unilateral
    problem.add_constraint(variable->expr(F_Z, 1) >= 0);

    // We want the ZMPs to remain in the contacts
    // We add constraints in the form of:
    // -l_1 f_x <= m_y <= l_1 f_x
    problem.add_constraint(variable->expr(M_Y, 1) <= ((length / 2) * variable->expr(F_Z, 1)));
    problem.add_constraint((-(length / 2) * variable->expr(F_Z, 1)) <= variable->expr(M_Y, 1));

    problem.add_constraint(variable->expr(M_X, 1) <= ((width / 2) * variable->expr(F_Z, 1)));
    problem.add_constraint((-(width / 2) * variable->expr(F_Z, 1)) <= variable->expr(M_X, 1));

    // We don't slip
    problem.add_constraint(variable->expr(F_X, 1) <= mu * variable->expr(F_Z, 1));
    problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_X, 1));

    problem.add_constraint(variable->expr(F_Y, 1) <= mu * variable->expr(F_Z, 1));
    problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_Y, 1));
  }

  // Objective
  if (weight_forces > 0)
  {
    problem.add_constraint(variable->expr(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);
  }
  if (weight_moments > 0)
  {
    problem.add_constraint(variable->expr(M_X, 3) == 0).configure(ProblemConstraint::Soft, weight_moments);
  }

  Contact::Wrench wrench;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, solver->N);
  J.block(0, 0, 3, solver->N) = position_task->A;
  J.block(3, 0, 3, solver->N) = orientation_task->A;

  wrench.J = J;
  wrench.f = variable->expr();

  return wrench;
}

ExternalWrenchContact::ExternalWrenchContact(RobotWrapper::FrameIndex frame_index) : frame_index(frame_index)
{
}

Contact::Wrench ExternalWrenchContact::add_wrench(Problem& problem)
{
  Contact::Wrench wrench;
  wrench.J = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL_WORLD_ALIGNED);
  wrench.f = Expression::from_vector(w_ext);
  return wrench;
}

PuppetContact::PuppetContact()
{
}

Contact::Wrench PuppetContact::add_wrench(Problem& problem)
{
  Contact::Wrench wrench;
  wrench.J = Eigen::MatrixXd(solver->N, solver->N);
  wrench.J.setIdentity();
  wrench.f = problem.add_variable(solver->N).expr();
  return wrench;
}
}  // namespace placo::dynamics