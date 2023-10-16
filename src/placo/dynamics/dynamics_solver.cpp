#include "placo/dynamics/dynamics_solver.h"
#include "placo/problem/problem.h"

namespace placo
{
namespace dynamics
{
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

PlanarContact& DynamicsSolver::add_planar_contact(FrameTask& frame_task)
{
  return add_contact(new PlanarContact(frame_task, true));
}

PlanarContact& DynamicsSolver::add_fixed_contact(FrameTask& frame_task)
{
  return add_contact(new PlanarContact(frame_task, false));
}

PositionTask& DynamicsSolver::add_position_task(pinocchio::FrameIndex frame_index, Eigen::Vector3d target_world)
{
  return add_task(new PositionTask(frame_index, target_world));
}

PositionTask& DynamicsSolver::add_position_task(std::string frame_name, Eigen::Vector3d target_world)
{
  return add_position_task(robot.get_frame_index(frame_name), target_world);
}

RelativePositionTask& DynamicsSolver::add_relative_position_task(pinocchio::FrameIndex frame_a_index,
                                                                 pinocchio::FrameIndex frame_b_index,
                                                                 Eigen::Vector3d target_world)
{
  return add_task(new RelativePositionTask(frame_a_index, frame_b_index, target_world));
}

RelativePositionTask& DynamicsSolver::add_relative_position_task(std::string frame_a_name, std::string frame_b_name,
                                                                 Eigen::Vector3d target_world)
{
  return add_relative_position_task(robot.get_frame_index(frame_a_name), robot.get_frame_index(frame_b_name),
                                    target_world);
}

CoMTask& DynamicsSolver::add_com_task(Eigen::Vector3d target_world)
{
  return add_task(new CoMTask(target_world));
}

StaticTask& DynamicsSolver::add_static_task()
{
  return add_task(new StaticTask());
}

JointsTask& DynamicsSolver::add_joints_task()
{
  return add_task(new JointsTask());
}

OrientationTask& DynamicsSolver::add_orientation_task(pinocchio::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
{
  return add_task(new OrientationTask(frame_index, R_world_frame));
}

OrientationTask& DynamicsSolver::add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame)
{
  return add_orientation_task(robot.get_frame_index(frame_name), R_world_frame);
}

FrameTask DynamicsSolver::add_frame_task(pinocchio::FrameIndex frame_index, Eigen::Affine3d T_world_frame)
{
  PositionTask& position = add_position_task(frame_index, T_world_frame.translation());
  OrientationTask& orientation = add_orientation_task(frame_index, T_world_frame.rotation());

  return FrameTask(&position, &orientation);
}

FrameTask DynamicsSolver::add_frame_task(std::string frame_name, Eigen::Affine3d T_world_frame)
{
  return add_frame_task(robot.get_frame_index(frame_name), T_world_frame);
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

void DynamicsSolver::enable_joint_limits(bool enable)
{
  joint_limits = enable;
}

void DynamicsSolver::enable_velocity_limits(bool enable)
{
  velocity_limits = enable;
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
    passive_ids.insert(robot.get_joint_v_offset(passive_joint));
  }

  if (torque_limits)
  {
    for (int k = 0; k < N - 6; k++)
    {
      problem.add_constraint(tau.slice(k + 6, 1) <= robot.model.effortLimit[k + 6]);
      problem.add_constraint(tau.slice(k + 6, 1) >= -robot.model.effortLimit[k + 6]);
    }
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
    e.A = Eigen::MatrixXd(constraints, N);
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
        e.A(constraint, k + 6) = dt;
        e.b(constraint) = -robot.model.velocityLimit[k + 6] + qd;
        constraint++;

        e.A(constraint, k + 6) = -dt;
        e.b(constraint) = -robot.model.velocityLimit[k + 6] - qd;
        constraint++;
      }

      if (joint_limits)
      {
        double qdd_max = 10.;  // XXX: This should be specified somewhere else
        double qd_max = sqrt((2 / 3.) * fmax(0., robot.model.upperPositionLimit[k + 7] - q) * qdd_max);
        e.A(constraint, k + 6) = dt;
        e.b(constraint) = qd - qd_max;
        constraint++;

        qd_max = sqrt((2 / 3.) * fabs(fmin(0., robot.model.lowerPositionLimit[k + 7] - q)) * qdd_max);
        e.A(constraint, k + 6) = -dt;
        e.b(constraint) = -qd - qd_max;
        constraint++;
      }
    }

    problem.add_constraint(e <= 0);
  }
}

DynamicsSolver::Result DynamicsSolver::solve()
{
  DynamicsSolver::Result result;
  std::vector<Variable*> contact_wrenches;

  problem.clear_constraints();
  problem.clear_variables();

  Variable& qdd = problem.add_variable(robot.model.nv);

  for (auto& task : tasks)
  {
    task->update();

    ProblemConstraint::Priority task_priority = ProblemConstraint::Hard;
    if (task->priority == Task::Priority::Soft)
    {
      task_priority = ProblemConstraint::Soft;
    }

    Expression e;
    e.A = task->A;
    e.b = -task->b;
    problem.add_constraint(e == 0).configure(task_priority, task->weight);
  }

  // We build the expression for tau, given the equation of motion
  // tau = M qdd + b - J^T F

  // We add some friction, this might be reworked and parametrized
  // Eigen::VectorXd friction = robot.state.qd * 1e-2;

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

  // Floating base has no torque
  problem.add_constraint(tau.slice(0, 6) == 0);

  // Passive joints have no torque
  for (auto& joint : robot.actuated_joint_names())
  {
    {
      if (passive_joints.count(joint) > 0)
      {
        problem.add_constraint(tau.slice(robot.get_joint_v_offset(joint), 1) == 0);
      }
    }
  }

  compute_limits_inequalities(tau);

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
  delete task;
}
}  // namespace dynamics
}  // namespace placo