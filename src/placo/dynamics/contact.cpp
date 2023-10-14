#include "placo/dynamics/contact.h"

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
Contact::Contact(RobotWrapper& robot) : robot(robot)
{
}

void Contact::configure(const std::string& frame_name, Contact::Type type, double mu, double length, double width)
{
  this->frame_name = frame_name;
  this->type = type;
  this->mu = mu;
  this->length = length;
  this->width = width;
}

Contact::Wrench Contact::add_wrench(RobotWrapper& robot, Problem& problem)
{
  if (frame_name == "")
  {
    throw std::runtime_error("Contact frame name is not set (did you call configure?)");
  }

  Wrench wrench;

  if (type == Fixed || type == Planar)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame_name, "local");
    // Wrench is: [ fx fy fz mx my mz ] with f the force and m the moment
    variable = &problem.add_variable(6);

    if (type == Planar)
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

    wrench.J = J;
    wrench.f = variable->expr();
  }
  else if (type == FixedPoint || type == Point)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame_name, "local_world_aligned").block(0, 0, 3, robot.model.nv);
    variable = &problem.add_variable(3);

    if (type == Point)
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

    wrench.J = J;
    wrench.f = variable->expr();
  }
  else
  {
    throw std::runtime_error("Unknown contact type");
  }

  return wrench;
}

Eigen::Vector3d Contact::zmp()
{
  if (type == Planar)
  {
    return Eigen::Vector3d(-wrench(M_Y, 0) / wrench(F_Z, 0), wrench(M_X, 0) / wrench(F_Z, 0), 0);
  }

  throw std::runtime_error("Contact ZMP is only defined for planar contacts");
}
}  // namespace dynamics
}  // namespace placo