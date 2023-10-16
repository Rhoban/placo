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

void DynamicsSolver::add_loop_closing_constraint(const std::string& frame_a, const std::string& frame_b,
                                                 const std::string& mask)
{
  LoopClosure constraint;
  constraint.frame_a = frame_a;
  constraint.frame_b = frame_b;
  constraint.mask.set_axises(mask);

  loop_closing_constraints.push_back(constraint);
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

PlanarContact& DynamicsSolver::add_planar_contact(PositionTask& position_task, OrientationTask& orientation_task)
{
  return add_contact(new PlanarContact(position_task, orientation_task, true));
}

PlanarContact& DynamicsSolver::add_fixed_contact(PositionTask& position_task, OrientationTask& orientation_task)
{
  return add_contact(new PlanarContact(position_task, orientation_task, false));
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

DynamicsSolver::Result DynamicsSolver::solve()
{
  DynamicsSolver::Result result;
  std::vector<Variable*> contact_wrenches;

  problem.clear_constraints();
  problem.clear_variables();

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
  for (auto& joint : robot.actuated_joint_names())
  {
    {
      if (passive_joints.count(joint) > 0)
      {
        problem.add_constraint(tau.slice(robot.get_joint_v_offset(joint), 1) == 0);
      }
      else
      {
        problem.add_constraint(tau.slice(robot.get_joint_v_offset(joint), 1) >= -1);
        problem.add_constraint(tau.slice(robot.get_joint_v_offset(joint), 1) <= 1);
      }
    }
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