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
using namespace placo::problem;

Contact::Contact()
{
}

Contact::~Contact()
{
}

int Contact::size()
{
  return J.rows();
}

void Contact::add_constraints(Problem& problem)
{
  // Forces minimization objective
  if (weight_forces > 0)
  {
    problem.add_constraint(f == 0).configure(ProblemConstraint::Soft, weight_forces);
  }
}

PointContact::PointContact(PositionTask& position_task, bool unilateral)
{
  this->position_task = &position_task;
  this->unilateral = unilateral;
  R_world_surface.setIdentity();
}

void PointContact::update()
{
  J = position_task->A;

  if (J.rows() != 3)
  {
    throw std::logic_error("PointContact::update: a point contact should be associated with a task of dimension 3 (did "
                           "you mask some axises?)");
  }
}

void PointContact::add_constraints(Problem& problem)
{
  if (unilateral)
  {
    // Expressing the force in the surface frame
    Expression f_surface = R_world_surface.transpose() * f;

    // The contact is unilateral
    problem.add_constraint(f_surface.slice(F_Z, 1) >= 0);

    // We don't slip
    problem.add_constraint(f_surface.slice(F_X, 1) <= mu * f_surface.slice(F_Z, 1));
    problem.add_constraint(-mu * f_surface.slice(F_Z, 1) <= f_surface.slice(F_X, 1));

    problem.add_constraint(f_surface.slice(F_Y, 1) <= mu * f_surface.slice(F_Z, 1));
    problem.add_constraint(-mu * f_surface.slice(F_Z, 1) <= f_surface.slice(F_Y, 1));
  }

  // Objective
  if (weight_forces > 0)
  {
    problem.add_constraint(f.slice(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);
  }
  if (weight_tangentials > 0)
  {
    Expression f_surface = R_world_surface.transpose() * f;
    problem.add_constraint(f_surface.slice(F_X, 2) == 0).configure(ProblemConstraint::Soft, weight_tangentials);
  }
}

Contact6D::Contact6D(FrameTask& frame_task, bool unilateral)
{
  this->position_task = frame_task.position;
  this->orientation_task = frame_task.orientation;
  this->unilateral = unilateral;
}

void Contact6D::update()
{
  J = Eigen::MatrixXd::Zero(6, solver->N);
  J.block(0, 0, 3, solver->N) =
      solver->robot.frame_jacobian(position_task->frame_index, pinocchio::LOCAL).block(0, 0, 3, solver->N);
  J.block(3, 0, 3, solver->N) =
      solver->robot.frame_jacobian(orientation_task->frame_index, pinocchio::LOCAL).block(3, 0, 3, solver->N);
}

void Contact6D::add_constraints(Problem& problem)
{
  if (unilateral)
  {
    if (length == 0 || width == 0.)
    {
      throw std::logic_error("Contact length and width should be set for unilateral planar contact");
    }

    // The contact is unilateral
    problem.add_constraint(f.slice(F_Z, 1) >= 0);

    // We want the ZMPs to remain in the contacts
    // We add constraints in the form of:
    // -l_1 f_x <= m_y <= l_1 f_x
    problem.add_constraint(f.slice(M_Y, 1) <= ((length / 2) * f.slice(F_Z, 1)));
    problem.add_constraint((-(length / 2) * f.slice(F_Z, 1)) <= f.slice(M_Y, 1));

    problem.add_constraint(f.slice(M_X, 1) <= ((width / 2) * f.slice(F_Z, 1)));
    problem.add_constraint((-(width / 2) * f.slice(F_Z, 1)) <= f.slice(M_X, 1));

    // We don't slip
    problem.add_constraint(f.slice(F_X, 1) <= mu * f.slice(F_Z, 1));
    problem.add_constraint(-mu * f.slice(F_Z, 1) <= f.slice(F_X, 1));

    problem.add_constraint(f.slice(F_Y, 1) <= mu * f.slice(F_Z, 1));
    problem.add_constraint(-mu * f.slice(F_Z, 1) <= f.slice(F_Y, 1));
  }

  // Objective
  if (weight_forces > 0)
  {
    problem.add_constraint(f.slice(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);
  }
  if (weight_tangentials > 0)
  {
    problem.add_constraint(f.slice(F_Y, 2) == 0).configure(ProblemConstraint::Soft, weight_tangentials);
  }
  if (weight_moments > 0)
  {
    problem.add_constraint(f.slice(M_X, 3) == 0).configure(ProblemConstraint::Soft, weight_moments);
  }
}

Eigen::Vector3d Contact6D::zmp()
{
  return Eigen::Vector3d(-wrench(M_Y, 0) / wrench(F_Z, 0), wrench(M_X, 0) / wrench(F_Z, 0), 0);
}

LineContact::LineContact(FrameTask& frame_task, bool unilateral)
{
  this->position_task = frame_task.position;
  this->orientation_task = frame_task.orientation;
  this->unilateral = unilateral;
  R_world_surface.setIdentity();
}

void LineContact::update()
{
  J = Eigen::MatrixXd::Zero(5, solver->N);
  J.block(0, 0, 3, solver->N) = solver->robot.frame_jacobian(position_task->frame_index, pinocchio::LOCAL_WORLD_ALIGNED)
                                    .block(0, 0, 3, solver->N);

  if (orientation_task->A.rows() != 2)
  {
    throw std::logic_error("LineContact::update: an orientation task should be associated with a task of dimension 2 "
                           "(you should have a yz mask on the orientation)");
  }

  J.block(3, 0, 2, solver->N) = orientation_task->A;
}

void LineContact::add_constraints(Problem& problem)
{
  if (unilateral)
  {
    if (length == 0)
    {
      throw std::logic_error("LineContact length should be set for unilateral contact");
    }

    // Expressing the force in the surface frame
    Expression f_surface = R_world_surface.transpose() * f;

    // The contact is unilateral
    problem.add_constraint(f_surface.slice(F_Z, 1) >= 0);

    // We want the ZMPs to remain in the contacts
    // We add constraints in the form of:
    // -l_1 f_x <= m_y <= l_1 f_x
    problem.add_constraint(f_surface.slice(M_Y, 1) <= ((length / 2) * f_surface.slice(F_Z, 1)));
    problem.add_constraint((-(length / 2) * f_surface.slice(F_Z, 1)) <= f_surface.slice(M_Y, 1));

    // We don't slip
    problem.add_constraint(f_surface.slice(F_X, 1) <= mu * f_surface.slice(F_Z, 1));
    problem.add_constraint(-mu * f_surface.slice(F_Z, 1) <= f_surface.slice(F_X, 1));

    problem.add_constraint(f_surface.slice(F_Y, 1) <= mu * f_surface.slice(F_Z, 1));
    problem.add_constraint(-mu * f_surface.slice(F_Z, 1) <= f_surface.slice(F_Y, 1));
  }

  // Objective
  if (weight_forces > 0)
  {
    Expression f_surface = R_world_surface.transpose() * f;
    problem.add_constraint(f_surface.slice(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);
  }
  if (weight_tangentials > 0)
  {
    Expression f_surface = R_world_surface.transpose() * f;
    problem.add_constraint(f_surface.slice(F_Y, 2) == 0).configure(ProblemConstraint::Soft, weight_tangentials);
  }
  if (weight_moments > 0)
  {
    // In a line contact, there are only two moments, which are actually M_Y and M_Z
    problem.add_constraint(f.slice(3, 2) == 0).configure(ProblemConstraint::Soft, weight_moments);
  }
}

Eigen::Vector3d LineContact::zmp()
{
  return Eigen::Vector3d(-wrench(M_Y, 0) / wrench(F_Z, 0), 0, 0);
}

ExternalWrenchContact::ExternalWrenchContact(model::RobotWrapper::FrameIndex frame_index,
                                             pinocchio::ReferenceFrame reference)
  : frame_index(frame_index), reference(reference)
{
}

void ExternalWrenchContact::update()
{
  J = solver->robot.frame_jacobian(frame_index, reference);
}

void ExternalWrenchContact::add_constraints(problem::Problem& problem)
{
  problem.add_constraint(f == w_ext);
}

PuppetContact::PuppetContact()
{
}

void PuppetContact::update()
{
  J = Eigen::MatrixXd(solver->N, solver->N);
  J.setIdentity();
}

TaskContact::TaskContact(Task& task)
{
  this->task = &task;
}

void TaskContact::update()
{
  J = task->A;
}

}  // namespace placo::dynamics