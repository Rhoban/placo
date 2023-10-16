#include "placo/dynamics/contacts.h"
#include "placo/dynamics/dynamics_solver.h"
#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"

// Some helpers for readability
#define F_X 0
#define F_Y 1
#define F_Z 2
#define M_X 3
#define M_Y 4
#define M_Z 5

namespace placo
{
namespace dynamics
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

Contact::Wrench PointContact::add_wrench(RobotWrapper& robot, Problem& problem)
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

PlanarContact::PlanarContact(PositionTask& position_task, OrientationTask& orientation_task, bool unilateral)
{
  this->position_task = &position_task;
  this->orientation_task = &orientation_task;
  this->unilateral = unilateral;
}

Contact::Wrench PlanarContact::add_wrench(RobotWrapper& robot, Problem& problem)
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
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, robot.model.nv);
  J.block(0, 0, 3, robot.model.nv) = position_task->A;
  J.block(3, 0, 3, robot.model.nv) = orientation_task->A;

  wrench.J = J;
  wrench.f = variable->expr();

  return wrench;
}

}  // namespace dynamics
}  // namespace placo