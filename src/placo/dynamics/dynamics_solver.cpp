#include "placo/dynamics/dynamics_solver.h"
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
namespace dynamics
{
void DynamicsSolver::Contact::configure(const std::string& frame_name, DynamicsSolver::Contact::Type type, double mu,
                                        double length, double width)
{
  this->frame_name = frame_name;
  this->type = type;
  this->mu = mu;
  this->length = length;
  this->width = width;
}

DynamicsSolver::Contact::Wrench DynamicsSolver::Contact::add_wrench(RobotWrapper& robot, Problem& problem)
{
  if (frame_name == "")
  {
    throw std::runtime_error("Contact frame name is not set (did you call configure?)");
  }

  Wrench wrench;

  if (type == Fixed)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame_name, "local");
    variable = &problem.add_variable(6);

    wrench.J = J;
    wrench.f = variable->expr();
  }
  else if (type == Planar)
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
  else if (type == Point)
  {
    Eigen::MatrixXd J = robot.frame_jacobian(frame_name, "local_world_aligned").block(0, 0, 3, robot.model.nv);
    variable = &problem.add_variable(3);

    // // The contact is unilateral
    // problem.add_constraint(variable->expr(F_Z, 1) >= 0);

    // // We don't slip
    // problem.add_constraint(variable->expr(F_X, 1) <= mu * variable->expr(F_Z, 1));
    // problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_X, 1));

    // problem.add_constraint(variable->expr(F_Y, 1) <= mu * variable->expr(F_Z, 1));
    // problem.add_constraint(-mu * variable->expr(F_Z, 1) <= variable->expr(F_Y, 1));

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

Eigen::Vector3d DynamicsSolver::Contact::zmp()
{
  if (type == Fixed)
  {
    return Eigen::Vector3d::Zero();
  }
  else if (type == Planar)
  {
    return Eigen::Vector3d(-wrench(M_Y, 0) / wrench(F_Z, 0), wrench(M_X, 0) / wrench(F_Z, 0), 0);
  }
  else if (type == Point)
  {
    return Eigen::Vector3d::Zero();
  }
  else
  {
    throw std::runtime_error("Unknown contact type");
  }
}

DynamicsSolver::Contact& DynamicsSolver::add_contact()
{
  contacts.push_back(new Contact());
  return *contacts.back();
}

void DynamicsSolver::set_passive(const std::string& joint_name, bool is_passive)
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

void DynamicsSolver::add_loop_closing_constraint(const std::string& frame_a, const std::string& frame_b,
                                                 const std::string& mask)
{
  LoopClosure constraint;
  constraint.frame_a = frame_a;
  constraint.frame_b = frame_b;
  constraint.mask.set_axises(mask);

  loop_closing_constraints.push_back(constraint);
}

PositionTask& DynamicsSolver::add_position_task(pinocchio::FrameIndex frame_index, Eigen::Vector3d target_world)
{
  return add_task(new PositionTask(frame_index, target_world));
}

PositionTask& DynamicsSolver::add_position_task(std::string frame_name, Eigen::Vector3d target_world)
{
  return add_position_task(robot.get_frame_index(frame_name), target_world);
}

OrientationTask& DynamicsSolver::add_orientation_task(pinocchio::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
{
  return add_task(new OrientationTask(frame_index, R_world_frame));
}

OrientationTask& DynamicsSolver::add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame)
{
  return add_orientation_task(robot.get_frame_index(frame_name), R_world_frame);
}

DynamicsSolver::DynamicsSolver(RobotWrapper& robot) : robot(robot)
{
  N = robot.model.nv;
}

DynamicsSolver::~DynamicsSolver()
{
  for (auto& contact : contacts)
  {
    delete contact;
  }
  for (auto& task : tasks)
  {
    delete task;
  }
}

DynamicsSolver::Result DynamicsSolver::solve()
{
  DynamicsSolver::Result result;
  std::vector<Variable*> contact_wrenches;

  Problem problem;
  Variable& qdd = problem.add_variable(robot.model.nv);

  if (qdd_desired.rows() == 0)
  {
    qdd_desired = Eigen::VectorXd::Zero(robot.model.nv);
  }

  // We impose the decision variable to be qdd_desired.
  // Later we can replace this with acceleration tasks.
  // problem.add_constraint(qdd.expr() == qdd_desired);

  for (auto& task : tasks)
  {
    task->update();

    ProblemConstraint::Priority task_priority = ProblemConstraint::Hard;
    if (task->priority == Task::Priority::Soft)
    {
      task_priority = ProblemConstraint::Soft;
    }

    std::cout << "Adding task " << task->type_name() << " with priority " << task_priority << std::endl;

    Expression e;
    e.A = task->A;
    e.b = -task->b;
    problem.add_constraint(e == 0).configure(task_priority, task->weight);
  }

  // We build the expression for tau, given the equation of motion
  // tau = M qdd + b - J^T F

  // M qdd
  Expression tau = robot.mass_matrix() * qdd.expr();

  // b
  tau = tau + robot.non_linear_effects();

  // J^T F
  // Computing body jacobians
  for (auto& contact : contacts)
  {
    Contact::Wrench wrench = contact->add_wrench(robot, problem);
    // problem.add_constraint(wrench.J * qdd.expr() == 0);
    tau = tau - wrench.J.transpose() * wrench.f;
  }

  // Loop closing constraints
  for (auto& constraint : loop_closing_constraints)
  {
    Eigen::MatrixXd J = robot.relative_position_jacobian(constraint.frame_a, constraint.frame_b)(
        constraint.mask.indices, Eigen::placeholders::all);
    Variable constraint_wrench = problem.add_variable(constraint.mask.indices.size());

    tau = tau - J.transpose() * constraint_wrench.expr();
  }

  // Floating base has no torque
  problem.add_constraint(tau.slice(0, 6) == 0);

  // Passive joints have no torque
  for (auto& joint_name : passive_joints)
  {
    problem.add_constraint(tau.slice(robot.get_joint_v_offset(joint_name), 1) == 0);
  }

  // We want to minimize torques
  problem.add_constraint(tau == 0).configure(ProblemConstraint::Soft, 1.0);

  try
  {
    // Solving the QP
    problem.solve();
    result.success = true;

    // Exporting result values
    result.tau = tau.value(problem.x);
    result.qdd = qdd.value;

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

void DynamicsSolver::remove_task(Task* task)
{
  tasks.erase(task);
  delete &task;
}
}  // namespace dynamics
}  // namespace placo