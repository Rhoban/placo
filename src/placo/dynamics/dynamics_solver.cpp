#include "placo/dynamics/dynamics_solver.h"
#include "placo/problem/problem.h"

namespace placo::dynamics
{
using namespace placo::problem;

void DynamicsSolver::set_passive(const std::string& joint_name, bool is_passive, double kp, double kd)
{
  if (!is_passive)
  {
    passive_joints.erase(joint_name);
  }
  else
  {
    PassiveJoint pj;
    pj.kp = kp;
    pj.kd = kd;
    passive_joints[joint_name] = pj;
  }
}

PointContact& DynamicsSolver::add_point_contact(PositionTask& position_task)
{
  return add_contact(new PointContact(position_task, false));
}

PointContact& DynamicsSolver::add_unilateral_point_contact(PositionTask& position_task)
{
  return add_contact(new PointContact(position_task, true));
}

RelativePointContact& DynamicsSolver::add_relative_point_contact(RelativePositionTask& position_task)
{
  return add_contact(new RelativePointContact(position_task));
}

Relative6DContact& DynamicsSolver::add_relative_fixed_contact(RelativeFrameTask& frame_task)
{
  return add_contact(new Relative6DContact(frame_task));
}

Contact6D& DynamicsSolver::add_planar_contact(FrameTask& frame_task)
{
  return add_contact(new Contact6D(frame_task, true));
}

Contact6D& DynamicsSolver::add_fixed_contact(FrameTask& frame_task)
{
  return add_contact(new Contact6D(frame_task, false));
}

ExternalWrenchContact& DynamicsSolver::add_external_wrench_contact(model::RobotWrapper::FrameIndex frame_index)
{
  return add_contact(new ExternalWrenchContact(frame_index));
}

ExternalWrenchContact& DynamicsSolver::add_external_wrench_contact(std::string frame_name)
{
  return add_external_wrench_contact(robot.get_frame_index(frame_name));
}

PuppetContact& DynamicsSolver::add_puppet_contact()
{
  return add_contact(new PuppetContact());
}

TaskContact& DynamicsSolver::add_task_contact(Task& task)
{
  return add_contact(new TaskContact(task));
}

AvoidSelfCollisionsConstraint& DynamicsSolver::add_avoid_self_collisions_constraint()
{
  return add_constraint(new AvoidSelfCollisionsConstraint());
}

ReactionRatioConstraint& DynamicsSolver::add_reaction_ratio_constraint(Contact& contact, double reaction_ratio)
{
  return add_constraint(new ReactionRatioConstraint(contact, reaction_ratio));
}

PositionTask& DynamicsSolver::add_position_task(model::RobotWrapper::FrameIndex frame_index,
                                                Eigen::Vector3d target_world)
{
  return add_task(new PositionTask(frame_index, target_world));
}

PositionTask& DynamicsSolver::add_position_task(std::string frame_name, Eigen::Vector3d target_world)
{
  return add_position_task(robot.get_frame_index(frame_name), target_world);
}

RelativePositionTask& DynamicsSolver::add_relative_position_task(model::RobotWrapper::FrameIndex frame_a_index,
                                                                 model::RobotWrapper::FrameIndex frame_b_index,
                                                                 Eigen::Vector3d target)
{
  return add_task(new RelativePositionTask(frame_a_index, frame_b_index, target));
}

RelativePositionTask& DynamicsSolver::add_relative_position_task(std::string frame_a_name, std::string frame_b_name,
                                                                 Eigen::Vector3d target)
{
  return add_relative_position_task(robot.get_frame_index(frame_a_name), robot.get_frame_index(frame_b_name), target);
}

RelativeOrientationTask& DynamicsSolver::add_relative_orientation_task(model::RobotWrapper::FrameIndex frame_a_index,
                                                                       model::RobotWrapper::FrameIndex frame_b_index,
                                                                       Eigen::Matrix3d R_a_b)
{
  return add_task(new RelativeOrientationTask(frame_a_index, frame_b_index, R_a_b));
}

RelativeOrientationTask& DynamicsSolver::add_relative_orientation_task(std::string frame_a_name,
                                                                       std::string frame_b_name, Eigen::Matrix3d R_a_b)
{
  return add_relative_orientation_task(robot.get_frame_index(frame_a_name), robot.get_frame_index(frame_b_name), R_a_b);
}

RelativeFrameTask DynamicsSolver::add_relative_frame_task(model::RobotWrapper::FrameIndex frame_a_index,
                                                          model::RobotWrapper::FrameIndex frame_b_index,
                                                          Eigen::Affine3d T_a_b)
{
  RelativePositionTask& position = add_relative_position_task(frame_a_index, frame_b_index, T_a_b.translation());
  RelativeOrientationTask& orientation = add_relative_orientation_task(frame_a_index, frame_b_index, T_a_b.rotation());

  return RelativeFrameTask(&position, &orientation);
}

RelativeFrameTask DynamicsSolver::add_relative_frame_task(std::string frame_a_name, std::string frame_b_name,
                                                          Eigen::Affine3d T_world_frame)
{
  return add_relative_frame_task(robot.get_frame_index(frame_a_name), robot.get_frame_index(frame_b_name),
                                 T_world_frame);
}

CoMTask& DynamicsSolver::add_com_task(Eigen::Vector3d target_world)
{
  return add_task(new CoMTask(target_world));
}

void DynamicsSolver::set_static(bool is_static_)
{
  is_static = is_static_;
}

JointsTask& DynamicsSolver::add_joints_task()
{
  return add_task(new JointsTask());
}

GearTask& DynamicsSolver::add_gear_task()
{
  return add_task(new GearTask());
}

OrientationTask& DynamicsSolver::add_orientation_task(model::RobotWrapper::FrameIndex frame_index,
                                                      Eigen::Matrix3d R_world_frame)
{
  return add_task(new OrientationTask(frame_index, R_world_frame));
}

OrientationTask& DynamicsSolver::add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame)
{
  return add_orientation_task(robot.get_frame_index(frame_name), R_world_frame);
}

FrameTask DynamicsSolver::add_frame_task(model::RobotWrapper::FrameIndex frame_index, Eigen::Affine3d T_world_frame)
{
  PositionTask& position = add_position_task(frame_index, T_world_frame.translation());
  OrientationTask& orientation = add_orientation_task(frame_index, T_world_frame.rotation());

  return FrameTask(&position, &orientation);
}

FrameTask DynamicsSolver::add_frame_task(std::string frame_name, Eigen::Affine3d T_world_frame)
{
  return add_frame_task(robot.get_frame_index(frame_name), T_world_frame);
}

DynamicsSolver::DynamicsSolver(model::RobotWrapper& robot) : robot(robot)
{
  N = robot.model.nv;
  masked_fbase = false;
  problem.use_sparsity = false;
  problem.rewrite_equalities = true;
}

DynamicsSolver::~DynamicsSolver()
{
  clear();
}

void DynamicsSolver::enable_joint_limits(bool enable)
{
  joint_limits = enable;
}

void DynamicsSolver::enable_velocity_limits(bool enable)
{
  velocity_limits = enable;
}

void DynamicsSolver::enable_velocity_vs_torque_limits(bool enable)
{
  velocity_limits = enable;
  velocity_vs_torque_limits = enable;
}

void DynamicsSolver::enable_torque_limits(bool enable)
{
  torque_limits = enable;
}

void DynamicsSolver::compute_limits_inequalities(Expression& tau)
{
  if ((joint_limits || velocity_limits) && dt == 0.)
  {
    throw std::runtime_error("DynamicsSolver::compute_limits_inequalities: dt is not set");
  }

  std::set<int> passive_ids;
  for (auto& passive_joint : passive_joints)
  {
    passive_ids.insert(robot.get_joint_v_offset(passive_joint.first));
  }

  if (torque_limits)
  {
    problem.add_constraint(tau.slice(6) <= robot.model.effortLimit.bottomRows(N - 6));
    problem.add_constraint(tau.slice(6) >= -robot.model.effortLimit.bottomRows(N - 6));
  }

  if (is_static)
  {
    return;
  }

  int constraints = 0;
  if (joint_limits)
  {
    constraints += 2 * (N - 6 - passive_joints.size());
  }
  if (velocity_limits)
  {
    constraints += 2 * (N - 6 - passive_joints.size());
  }

  if (constraints > 0)
  {
    Expression e;
    e.A = Eigen::MatrixXd(constraints, problem.n_variables);
    e.A.setZero();
    e.b = Eigen::VectorXd(constraints);
    int constraint = 0;

    // Iterating for each actuated joints
    for (int k = 0; k < N - 6; k++)
    {
      if (passive_ids.count(k + 6) > 0)
      {
        continue;
      }
      double q = robot.state.q[k + 7];
      double qd = robot.state.qd[k + 6];

      if (velocity_limits)
      {
        if (torque_limits && velocity_vs_torque_limits)
        {
          double ratio = robot.model.velocityLimit[k + 6] / robot.model.effortLimit[k + 6];

          // qd + dt*qdd <= qd_max - ratio * tau
          // ratio * tau + dt*qdd + qd - qd_max <= 0
          e.A.block(constraint, 0, 1, problem.n_variables) = ratio * tau.A.block(k + 6, 0, 1, problem.n_variables);
          e.b[constraint] = ratio * tau.b[k + 6];
          e.A(constraint, k + 6) += dt;
          e.b[constraint] += qd - robot.model.velocityLimit[k + 6];
          constraint++;

          // qd + dt*qdd >= -qd_max - ratio * tau
          // -ratio*tau - dt*qdd - qd - qd_max <= 0
          e.A.block(constraint, 0, 1, problem.n_variables) = -ratio * tau.A.block(k + 6, 0, 1, problem.n_variables);
          e.b[constraint] = -ratio * tau.b[k + 6];
          e.A(constraint, k + 6) -= dt;
          e.b[constraint] -= qd + robot.model.velocityLimit[k + 6];
          constraint++;
        }
        else
        {
          e.A(constraint, k + 6) = dt;
          e.b(constraint) = -robot.model.velocityLimit[k + 6] + qd;
          constraint++;

          e.A(constraint, k + 6) = -dt;
          e.b(constraint) = -robot.model.velocityLimit[k + 6] - qd;
          constraint++;
        }
      }

      if (joint_limits)
      {
        if (q > robot.model.upperPositionLimit[k + 7])
        {
          // We are in the contact, ensuring at least
          // qdd <= -qdd_safe
          e.A(constraint, k + 6) = 1;
          e.b(constraint) = qdd_safe;
        }
        else
        {
          // qdd*dt + qd <= qd_max
          double qd_max = sqrt(2. * (robot.model.upperPositionLimit[k + 7] - q) * qdd_safe);
          e.A(constraint, k + 6) = dt;
          e.b(constraint) = qd - qd_max;
        }
        constraint++;

        if (q < robot.model.lowerPositionLimit[k + 7])
        {
          // We are in the contact, ensuring at least
          // qdd >= qdd_safe
          e.A(constraint, k + 6) = -1;
          e.b(constraint) = qdd_safe;
        }
        else
        {
          // qdd*dt + qd >= -qd_max
          double qd_max = sqrt(2. * fabs(robot.model.lowerPositionLimit[k + 7] - q) * qdd_safe);
          e.A(constraint, k + 6) = -dt;
          e.b(constraint) = -qd - qd_max;
        }
        constraint++;
      }
    }

    problem.add_constraint(e <= 0);
  }
}

void DynamicsSolver::clear()
{
  for (auto& task : tasks)
  {
    if (task->solver_memory)
    {
      delete task;
    }
  }
  tasks.clear();

  for (auto& constraint : constraints)
  {
    if (constraint->solver_memory)
    {
      delete constraint;
    }
  }
  constraints.clear();

  for (auto& contact : contacts)
  {
    if (contact->solver_memory)
    {
      delete contact;
    }
  }
  contacts.clear();
}

void DynamicsSolver::dump_status_stream(std::ostream& stream)
{
  stream << "* Dynamics Tasks:" << std::endl;
  if (is_static)
  {
    std::cout << "  * Solver is static (qdd is 0)" << std::endl;
  }
  for (auto task : tasks)
  {
    task->update();
    stream << "  * " << task->name << " [" << task->type_name() << "]" << std::endl;
    stream << "    - Priority: ";
    if (task->priority == Task::Priority::Hard)
    {
      stream << "hard";
    }
    else
    {
      stream << "soft (weight:" << task->weight << ")";
    }
    stream << std::endl;
    char buffer[128];
    sprintf(buffer, "    - Error: %.06f [%s]\n", task->error.norm(), task->error_unit().c_str());
    stream << buffer << std::endl;
    sprintf(buffer, "    - DError: %.06f [%s]\n", task->derror.norm(), task->error_unit().c_str());
    stream << buffer << std::endl;
  }
}

void DynamicsSolver::dump_status()
{
  dump_status_stream(std::cout);
}

DynamicsSolver::Result DynamicsSolver::solve(bool integrate)
{
  DynamicsSolver::Result result;
  std::vector<Variable*> contact_wrenches;

  problem.clear_constraints();
  problem.clear_variables();

  // Computing target torque for passive joints
  std::vector<int> passive_indices;
  Eigen::VectorXd passive_taus = Eigen::VectorXd::Zero(passive_joints.size());
  int k = 0;
  for (auto& entry : passive_joints)
  {
    std::string joint = entry.first;
    PassiveJoint pj = entry.second;
    double q = robot.get_joint(joint);
    double qd = robot.get_joint_velocity(joint);
    int index = robot.get_joint_v_offset(joint);
    passive_indices.push_back(index);
    passive_taus[k++] = q * pj.kp + qd * pj.kd;
  }

  Expression qdd;
  if (is_static)
  {
    qdd = Expression::from_vector(Eigen::VectorXd::Zero(robot.model.nv));
  }
  else
  {
    Variable& qdd_variable = problem.add_variable(robot.model.nv);
    qdd = qdd_variable.expr();

    if (masked_fbase)
    {
      problem.add_constraint(qdd_variable.expr(0, 6) == 0.);
    }
  }

  for (auto& task : tasks)
  {
    task->update();
    if (task->A.rows() == 0)
    {
      continue;
    }

    if (!is_static)
    {
      ProblemConstraint::Priority task_priority = ProblemConstraint::Hard;
      if (task->priority == Task::Priority::Soft)
      {
        task_priority = ProblemConstraint::Soft;
      }
      else if (task->priority == Task::Priority::Scaled)
      {
        throw std::runtime_error("DynamicsSolver::solve: Scaled priority is not supported");
      }

      Expression e;
      e.A = task->A;
      e.b = -task->b;
      problem.add_constraint(e == 0).configure(task_priority, task->weight);
    }
  }

  // We build the expression for tau, given the equation of motion
  // tau = M qdd + b - J^T F

  // M qdd
  Expression tau = robot.mass_matrix() * qdd + robot.state.qd * friction;

  // b
  if (gravity_only)
  {
    tau = tau + robot.generalized_gravity();
  }
  else
  {
    tau = tau + robot.non_linear_effects();
  }

  // J^T F

  // Contacts that will result in a decision variable
  std::vector<Contact*> variable_contacts;
  // Contacts that will be optimized out
  std::vector<Contact*> determined_contacts;
  std::vector<int> determined_indices;
  int determined_contacts_count = 0;

  for (auto& contact : contacts)
  {
    contact->update();

    ExternalWrenchContact* ext = dynamic_cast<ExternalWrenchContact*>(contact);
    if (ext != nullptr)
    {
      Eigen::VectorXd w_ext = ext->w_ext;
      tau.b -= contact->J.transpose() * w_ext;
    }
    else
    {
      if (optimize_contact_forces && contact->is_internal() &&
          determined_contacts_count + contact->size() <= passive_joints.size())
      {
        // This contact will be determined
        determined_contacts.push_back(contact);
        determined_contacts_count += contact->size();
      }
      else
      {
        // This contact will be an actual decision variable
        Variable& f_variable = problem.add_variable(contact->size());
        contact->f = f_variable.expr();
        contact->add_constraints(problem);
        variable_contacts.push_back(contact);
      }
    }
  }

  // The number of decision variables is now known, resizing the expression of tau
  int cols_before = tau.A.cols();
  tau.A.conservativeResize(N, problem.n_variables);
  tau.A.block(0, cols_before, N, problem.n_variables - cols_before).setZero();

  // Now, tau = Ax + b with x = [qdd, f1, f2, ...], we copy J^T to the extended A
  // for forces that are decision variables
  k = is_static ? 0 : N;
  for (auto& contact : variable_contacts)
  {
    tau.A.block(0, k, N, contact->J.rows()) = -contact->J.transpose();
    tau.b -= contact->J.transpose() * contact->f.b;
    k += contact->J.rows();
  }

  // Gathering the first N entries of the passive torque ids
  determined_indices = std::vector<int>(passive_indices.begin(), passive_indices.begin() + determined_contacts_count);

  if (optimize_contact_forces && determined_contacts_count > 0)
  {
    // Computing Jd, the jacobian of determined contact forces
    Eigen::MatrixXd Jd = Eigen::MatrixXd::Zero(N, determined_contacts_count);
    k = 0;
    for (auto& contact : determined_contacts)
    {
      Jd.block(0, k, N, contact->J.rows()) = contact->J.transpose();
      k += contact->J.rows();
    }

    // Building the expression for the determined forces
    // Jd^T fd = M qdd + b - J^T F - tau_passive
    Expression fd;
    fd.A = tau.A(determined_indices, Eigen::all);
    fd.b = tau.b(determined_indices);
    fd.b -= passive_taus.topRows(determined_contacts_count);

    // We use the inverse of the Jd matrix for those passive Dofs to express fd as a
    // function of other variables
    fd = Jd(determined_indices, Eigen::all).inverse() * fd;

    // We feed the determined contacts with force expressed as other variables
    k = 0;
    for (auto& contact : determined_contacts)
    {
      contact->f = fd.slice(k, contact->size());
      contact->add_constraints(problem);
      k += contact->size();
    }

    // fd can now be added to tau
    tau = tau - Jd * fd;
  }

  // Computing limit inequalitie
  compute_limits_inequalities(tau);

  // Add constraints
  for (auto constraint : constraints)
  {
    constraint->add_constraint(problem, tau);
  }

  // Floating base has no torque, except if is masked (in that case, the floating base torque will
  // allow to compensate for any motion)
  if (!masked_fbase)
  {
    problem.add_constraint(tau.slice(0, 6) == 0);
  }

  // Passive joints that are not determined have a tau constraint
  if (passive_taus.size() > determined_contacts_count)
  {
    Expression passive_tau_expr;
    passive_tau_expr.A = tau.A(passive_indices, Eigen::all);
    passive_tau_expr.b = tau.b(passive_indices);
    problem.add_constraint(passive_tau_expr.slice(determined_contacts_count) ==
                           passive_taus.bottomRows(passive_taus.size() - determined_contacts_count));
  }

  // We want to minimize torques
  problem.add_constraint(tau == 0).configure(ProblemConstraint::Soft, 1e-3);

  try
  {
    // Solving the QP
    problem.solve();
    result.success = true;

    // Exporting result values
    result.tau = tau.value(problem.x);
    result.qdd = qdd.value(problem.x);

    for (auto& contact : contacts)
    {
      contact->wrench = contact->f.value(problem.x);
    }

    if (integrate)
    {
      if (dt == 0.)
      {
        throw std::runtime_error("DynamicsSolver::solve, trying to integrate, but dt is not set");
      }

      robot.state.qdd = result.qdd;
      robot.integrate(dt);
    }
  }
  catch (QPError& e)
  {
    result.success = false;
  }

  return result;
}

void DynamicsSolver::mask_fbase(bool masked)
{
  masked_fbase = masked;
}

void DynamicsSolver::remove_task(Task& task)
{
  tasks.erase(&task);

  delete &task;
}

void DynamicsSolver::remove_task(FrameTask& task)
{
  remove_task(*task.position);
  remove_task(*task.orientation);
}

void DynamicsSolver::remove_contact(Contact& contact)
{
  // Removing the contact from the vector
  contacts.erase(std::remove(contacts.begin(), contacts.end(), &contact), contacts.end());

  if (contact.solver_memory)
  {
    delete &contact;
  }
}

void DynamicsSolver::remove_constraint(Constraint& constraint)
{
  constraints.erase(&constraint);

  if (constraint.solver_memory)
  {
    delete &constraint;
  }
}

void DynamicsSolver::add_task(Task& task)
{
  task.solver = this;
  tasks.insert(&task);
}

void DynamicsSolver::add_constraint(Constraint& constraint)
{
  constraint.solver = this;
  constraints.insert(&constraint);
}

void DynamicsSolver::add_contact(Contact& contact)
{
  contact.solver = this;
  contacts.push_back(&contact);
}

}  // namespace placo::dynamics