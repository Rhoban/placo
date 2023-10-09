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
GravityTorques::Contact::Contact(const std::string& frame_name, Type type, double mu, double length, double width)
  : frame_name(frame_name), type(type), mu(mu), length(length), width(width)
{
}

void GravityTorques::add_contact(Contact contact)
{
  contacts.push_back(contact);
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

GravityTorques::Result GravityTorques::compute_gravity_torques(RobotWrapper& robot)
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

  // Computing body jacobians
  for (auto& contact : contacts)
  {
  }

  for (auto& frame : unilateral_contacts)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame, "local");

    // Wrench is: [ fx fy fz mx my mz ] with f the force and m the moment
    Variable& wrench = problem.add_variable(6);
    contact_wrenches.push_back(&wrench);

    torque_forces = torque_forces + J.transpose() * wrench.expr();
  }
  problem.add_constraint(torque_forces == robot.generalized_gravity());

  // // z component of forces should be positive
  // for (auto wrench : contact_wrenches)
  // {
  //   problem.add_constraint(wrench->expr(F_Z, 1) >= 0);
  // }

  // // We want the ZMPs to remain in the contacts
  // // We add constraints in the form of:
  // // -l_1 f_x <= m_y <= l_1 f_x
  // for (auto wrench : contact_wrenches)
  // {
  //   problem.add_constraint(wrench->expr(M_Y, 1) <= ((contact_length / 2) * wrench->expr(F_Z, 1)));
  //   problem.add_constraint((-(contact_length / 2) * wrench->expr(F_Z, 1)) <= wrench->expr(M_Y, 1));

  //   problem.add_constraint(wrench->expr(M_X, 1) <= ((contact_width / 2) * wrench->expr(F_Z, 1)));
  //   problem.add_constraint((-(contact_width / 2) * wrench->expr(F_Z, 1)) <= wrench->expr(M_X, 1));
  // }

  // // We can't slip on the ground
  // for (auto wrench : contact_wrenches)
  // {
  //   problem.add_constraint(wrench->expr(F_X, 1) <= mu * wrench->expr(F_Z, 1));
  //   problem.add_constraint(-mu * wrench->expr(F_Z, 1) <= wrench->expr(F_X, 1));

  //   problem.add_constraint(wrench->expr(F_Y, 1) <= mu * wrench->expr(F_Z, 1));
  //   problem.add_constraint(-mu * wrench->expr(F_Z, 1) <= wrench->expr(F_Y, 1));
  // }

  // // We want to minimize moments in x and y
  // for (auto wrench : contact_wrenches)
  // {
  //   problem.add_constraint(wrench->expr(M_X, 1) == 0).configure(ProblemConstraint::Soft, 100.0);
  //   problem.add_constraint(wrench->expr(M_Y, 1) == 0).configure(ProblemConstraint::Soft, 100.0);
  // }

  // // We want to minimize torques
  // problem.add_constraint(tau.expr() == 0).configure(ProblemConstraint::Soft, 1.0);

  // try
  // {
  //   // Solving the QP
  //   problem.solve();
  //   result.success = true;

  //   // Exporting result values
  //   result.tau = tau.value;
  //   result.contact_wrenches = Eigen::MatrixXd(contact_wrenches.size(), 6);
  //   int k = 0;
  //   for (auto wrench : contact_wrenches)
  //   {
  //     result.contact_wrenches.block(k, 0, 1, 6) = wrench->value.transpose();
  //     k += 1;
  //   }
  // }
  // catch (QPError& e)
  // {
  //   result.success = false;
  // }

  return result;
}
}  // namespace placo