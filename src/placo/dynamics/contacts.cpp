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
}

bool Contact::is_internal()
{
  return false;
}

Eigen::Vector3d Contact6D::zmp()
{
  return Eigen::Vector3d(-wrench(M_Y, 0) / wrench(F_Z, 0), wrench(M_X, 0) / wrench(F_Z, 0), 0);
}

PointContact::PointContact(PositionTask& position_task, bool unilateral)
{
  this->position_task = &position_task;
  this->unilateral = unilateral;
}

void PointContact::update()
{
  J = position_task->A;
}

void PointContact::add_constraints(Problem& problem)
{
  if (unilateral)
  {
    // The contact is unilateral
    problem.add_constraint(f.slice(F_Z, 1) >= 0);

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
}

bool RelativePointContact::is_internal()
{
  return true;
}

RelativePointContact::RelativePointContact(RelativePositionTask& relative_position_task)
{
  this->relative_position_task = &relative_position_task;
}

void RelativePointContact::update()
{
  J = relative_position_task->A;
}

void RelativePointContact::add_constraints(Problem& problem)
{
  // Objective
  if (weight_forces > 0)
  {
    problem.add_constraint(f == 0).configure(ProblemConstraint::Soft, weight_forces);
  }
}

Relative6DContact::Relative6DContact(RelativeFrameTask& relative_frame_task)
{
  this->relative_position_task = relative_frame_task.position;
  this->relative_orientation_task = relative_frame_task.orientation;
}

void Relative6DContact::update()
{
  J = Eigen::MatrixXd::Zero(6, solver->N);
  J.block(0, 0, 3, solver->N) =
      solver->robot.frame_jacobian(relative_position_task->frame_b_index, pinocchio::WORLD).block(0, 0, 3, solver->N) -
      solver->robot.frame_jacobian(relative_position_task->frame_a_index, pinocchio::WORLD).block(0, 0, 3, solver->N);
  J.block(3, 0, 3, solver->N) = solver->robot.frame_jacobian(relative_orientation_task->frame_b_index, pinocchio::WORLD)
                                    .block(3, 0, 3, solver->N) -
                                solver->robot.frame_jacobian(relative_orientation_task->frame_a_index, pinocchio::WORLD)
                                    .block(3, 0, 3, solver->N);
}

bool Relative6DContact::is_internal()
{
  return true;
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
  if (weight_moments > 0)
  {
    problem.add_constraint(f.slice(M_X, 3) == 0).configure(ProblemConstraint::Soft, weight_moments);
  }
}

ExternalWrenchContact::ExternalWrenchContact(model::RobotWrapper::FrameIndex frame_index) : frame_index(frame_index)
{
}

void ExternalWrenchContact::update()
{
  J = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL_WORLD_ALIGNED);
}

PuppetContact::PuppetContact()
{
}

void PuppetContact::update()
{
  J = Eigen::MatrixXd(solver->N, solver->N);
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