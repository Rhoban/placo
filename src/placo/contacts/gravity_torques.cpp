#include "placo/contacts/gravity_torques.h"
#include "placo/problem/problem.h"

// Some helpers for readability
#define F_X 0
#define F_Y 1
#define F_Z 2
#define M_X 3
#define M_Y 4
#define M_Z 5

namespace placo
{
void GravityTorques::Contact::configure(const std::string& frame_name, GravityTorques::Contact::Type type, double mu,
                                        double length, double width)
{
  this->frame_name = frame_name;
  this->type = type;
  this->mu = mu;
  this->length = length;
  this->width = width;
}

Expression GravityTorques::Contact::add_wrench(RobotWrapper& robot, Problem& problem)
{
  if (frame_name == "")
  {
    throw std::runtime_error("Contact frame name is not set (did you call configure?)");
  }

  if (type == Planar)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame_name, "local");

    // Wrench is: [ fx fy fz mx my mz ] with f the force and m the moment
    variable = &problem.add_variable(6);

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

    // Objective
    problem.add_constraint(variable->expr(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);
    problem.add_constraint(variable->expr(M_X, 3) == 0).configure(ProblemConstraint::Soft, weight_moments);

    return J.transpose() * variable->expr();
  }
  else if (type == Point)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame_name, "local_world_aligned").block(0, 0, 3, robot.model.nv);
    variable = &problem.add_variable(3);

    // The contact is unilateral
    problem.add_constraint(variable->expr(F_Z, 1) >= 0);

    // We don't slip
    problem.add_constraint(variable->expr(F_X, 1) <= mu * variable->expr(F_Z, 1));
    problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_X, 1));

    problem.add_constraint(variable->expr(F_Y, 1) <= mu * variable->expr(F_Z, 1));
    problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_Y, 1));

    // Objective
    problem.add_constraint(variable->expr(F_X, 3) == 0).configure(ProblemConstraint::Soft, weight_forces);

    return J.transpose() * variable->expr();
  }
  else
  {
    throw std::runtime_error("Unknown contact type");
  }
}

GravityTorques::Contact& GravityTorques::add_contact()
{
  contacts.push_back(new Contact());
  return *contacts.back();
}

void GravityTorques::set_passive(const std::string& joint_name, bool is_passive)
{
  if (!is_passive)
  {
    passive_joints.erase(joint_name);
  }
  else
  {
    passive_joints.insert(joint_name);
  }
}

void GravityTorques::add_loop_closing_constraint(const std::string& frame_a, const std::string& frame_b)
{
  loop_closing_constraints.push_back({ frame_a, frame_b });
}

GravityTorques::GravityTorques(RobotWrapper& robot) : robot(robot)
{
}

GravityTorques::~GravityTorques()
{
  for (auto& contact : contacts)
  {
    delete contact;
  }
}

GravityTorques::Result GravityTorques::compute()
{
  GravityTorques::Result result;
  std::vector<Variable*> contact_wrenches;

  Problem problem;
  Variable& tau = problem.add_variable(robot.model.nv);

  // Floating base has no torque
  problem.add_constraint(tau.expr(0, 6) == 0);

  // Passive joints have no torque
  for (auto& joint_name : passive_joints)
  {
    problem.add_constraint(tau.expr(robot.get_joint_v_offset(joint_name), 1) == 0);
  }

  // We must satisfy the equation of motion with contact unilateral forces
  // tau + sum(J^T forces) = g
  Expression torque_forces = tau.expr();
  std::map<Contact*, Variable*> contact_wrenches_map;

  // Computing body jacobians
  for (auto& contact : contacts)
  {
    torque_forces = torque_forces + contact->add_wrench(robot, problem);
  }

  // Loop closing constraints
  for (auto& constraint : loop_closing_constraints)
  {
    Eigen::MatrixXd J = robot.relative_position_jacobian(constraint.first, constraint.second);
    Variable constraint_wrench = problem.add_variable(2);
    torque_forces = torque_forces + J.transpose() * constraint_wrench.expr();
  }

  // Equation of motion
  problem.add_constraint(torque_forces == robot.generalized_gravity());

  // We want to minimize torques
  problem.add_constraint(tau.expr() == 0).configure(ProblemConstraint::Soft, 1.0);

  try
  {
    // Solving the QP
    problem.solve();
    result.success = true;

    // Exporting result values
    result.tau = tau.value;

    // Updating wrenches
    for (auto& contact : contacts)
    {
      contact->wrench = contact->variable->value;
    }
  }
  catch (QPError& e)
  {
    result.success = false;
  }

  return result;
}
}  // namespace placo