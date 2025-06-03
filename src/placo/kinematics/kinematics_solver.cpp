#include "placo/kinematics/kinematics_solver.h"
#include "eiquadprog/eiquadprog.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"
#include "placo/tools/utils.h"

namespace placo::kinematics
{
using namespace placo::problem;

KinematicsSolver::KinematicsSolver(model::RobotWrapper& robot_) : robot(robot_), masked_fbase(false)
{
  N = robot.model.nv;
  problem.use_sparsity = false;
}

KinematicsSolver::~KinematicsSolver()
{
  clear();
}

PositionTask& KinematicsSolver::add_position_task(model::RobotWrapper::FrameIndex frame, Eigen::Vector3d target_world)
{
  return add_task(new PositionTask(frame, target_world));
}

PositionTask& KinematicsSolver::add_position_task(std::string frame, Eigen::Vector3d target_world)
{
  return add_position_task(robot.get_frame_index(frame), target_world);
}

RelativePositionTask& KinematicsSolver::add_relative_position_task(model::RobotWrapper::FrameIndex frame_a,
                                                                   model::RobotWrapper::FrameIndex frame_b,
                                                                   Eigen::Vector3d target)
{
  return add_task(new RelativePositionTask(frame_a, frame_b, target));
}

RelativePositionTask& KinematicsSolver::add_relative_position_task(std::string frame_a, std::string frame_b,
                                                                   Eigen::Vector3d target)
{
  return add_relative_position_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), target);
}

CoMTask& KinematicsSolver::add_com_task(Eigen::Vector3d targetCom_world)
{
  return add_task(new CoMTask(targetCom_world));
}

OrientationTask& KinematicsSolver::add_orientation_task(model::RobotWrapper::FrameIndex frame,
                                                        Eigen::Matrix3d R_world_frame)
{
  return add_task(new OrientationTask(frame, R_world_frame));
}

OrientationTask& KinematicsSolver::add_orientation_task(std::string frame, Eigen::Matrix3d R_world_frame)
{
  return add_orientation_task(robot.get_frame_index(frame), R_world_frame);
}

RelativeOrientationTask& KinematicsSolver::add_relative_orientation_task(model::RobotWrapper::FrameIndex frame_a,
                                                                         model::RobotWrapper::FrameIndex frame_b,
                                                                         Eigen::Matrix3d R_a_b)
{
  return add_task(new RelativeOrientationTask(frame_a, frame_b, R_a_b));
}

RelativeOrientationTask& KinematicsSolver::add_relative_orientation_task(std::string frame_a, std::string frame_b,
                                                                         Eigen::Matrix3d R_a_b)
{
  return add_relative_orientation_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), R_a_b);
}

FrameTask KinematicsSolver::add_frame_task(model::RobotWrapper::FrameIndex frame, Eigen::Affine3d T_world_frame)
{
  PositionTask& position = add_position_task(frame, T_world_frame.translation());
  OrientationTask& orientation = add_orientation_task(frame, T_world_frame.rotation());

  return FrameTask(&position, &orientation);
}

FrameTask KinematicsSolver::add_frame_task(std::string frame, Eigen::Affine3d T_world_frame)
{
  return add_frame_task(robot.get_frame_index(frame), T_world_frame);
}

RelativeFrameTask KinematicsSolver::add_relative_frame_task(model::RobotWrapper::FrameIndex frame_a,
                                                            model::RobotWrapper::FrameIndex frame_b,
                                                            Eigen::Affine3d T_a_b)
{
  RelativePositionTask& position = add_relative_position_task(frame_a, frame_b, T_a_b.translation());
  RelativeOrientationTask& orientation = add_relative_orientation_task(frame_a, frame_b, T_a_b.rotation());

  return RelativeFrameTask(position, orientation);
}

RelativeFrameTask KinematicsSolver::add_relative_frame_task(std::string frame_a, std::string frame_b,
                                                            Eigen::Affine3d T_a_b)
{
  return add_relative_frame_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), T_a_b);
}

AxisAlignTask& KinematicsSolver::add_axisalign_task(model::RobotWrapper::FrameIndex frame, Eigen::Vector3d axis_frame,
                                                    Eigen::Vector3d targetAxis_world)
{
  return add_task(new AxisAlignTask(frame, axis_frame, targetAxis_world));
}

AxisAlignTask& KinematicsSolver::add_axisalign_task(std::string frame, Eigen::Vector3d axis_frame,
                                                    Eigen::Vector3d target_axis_world)
{
  return add_axisalign_task(robot.get_frame_index(frame), axis_frame, target_axis_world);
}

JointsTask& KinematicsSolver::add_joints_task(std::map<std::string, double>& joints)
{
  JointsTask& task = add_task(new JointsTask());
  for (auto& entry : joints)
  {
    task.joints[entry.first] = entry.second;
  }
  return task;
}

DistanceTask& KinematicsSolver::add_distance_task(model::RobotWrapper::FrameIndex frame_a,
                                                  model::RobotWrapper::FrameIndex frame_b, double distance)
{
  return add_task(new DistanceTask(frame_a, frame_b, distance));
}

DistanceTask& KinematicsSolver::add_distance_task(std::string frame_a, std::string frame_b, double distance)
{
  return add_distance_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), distance);
}

CentroidalMomentumTask& KinematicsSolver::add_centroidal_momentum_task(Eigen::Vector3d L_world)
{
  return add_task(new CentroidalMomentumTask(L_world));
}

JointsTask& KinematicsSolver::add_joints_task()
{
  return add_task(new JointsTask());
}

GearTask& KinematicsSolver::add_gear_task()
{
  return add_task(new GearTask());
}

WheelTask& KinematicsSolver::add_wheel_task(std::string joint, double radius, bool omniwheel)
{
  return add_task(new WheelTask(joint, radius, omniwheel));
}

RegularizationTask& KinematicsSolver::add_regularization_task(double magnitude)
{
  RegularizationTask& task = add_task(new RegularizationTask());
  task.configure("regularization", Task::Priority::Soft, magnitude);

  return task;
}

ManipulabilityTask& KinematicsSolver::add_manipulability_task(model::RobotWrapper::FrameIndex frame,
                                                              ManipulabilityTask::Type type, double lambda)
{
  return add_task(new ManipulabilityTask(frame, type, lambda));
}

ManipulabilityTask& KinematicsSolver::add_manipulability_task(std::string frame, std::string type, double lambda)
{
  ManipulabilityTask::Type type_;
  if (type == "position")
  {
    type_ = ManipulabilityTask::Type::POSITION;
  }
  else if (type == "orientation")
  {
    type_ = ManipulabilityTask::Type::ORIENTATION;
  }
  else if (type == "both")
  {
    type_ = ManipulabilityTask::Type::BOTH;
  }
  else
  {
    throw std::runtime_error("Unknown manipulability type: " + type);
  }

  return add_manipulability_task(robot.get_frame_index(frame), type_, lambda);
}

KineticEnergyRegularizationTask& KinematicsSolver::add_kinetic_energy_regularization_task(double magnitude)
{
  KineticEnergyRegularizationTask& task = add_task(new KineticEnergyRegularizationTask());
  task.configure("kinetic_energy_regularization", Task::Priority::Soft, magnitude);

  return task;
}

AvoidSelfCollisionsConstraint& KinematicsSolver::add_avoid_self_collisions_constraint()
{
  return add_constraint(new AvoidSelfCollisionsConstraint());
}

CoMPolygonConstraint& KinematicsSolver::add_com_polygon_constraint(std::vector<Eigen::Vector2d> polygon, double margin)
{
  return add_constraint(new CoMPolygonConstraint(polygon, margin));
}

JointSpaceHalfSpacesConstraint& KinematicsSolver::add_joint_space_half_spaces_constraint(Eigen::MatrixXd A,
                                                                                         Eigen::VectorXd b)
{
  return add_constraint(new JointSpaceHalfSpacesConstraint(A, b));
}

ConeConstraint& KinematicsSolver::add_cone_constraint(model::RobotWrapper::FrameIndex frame_a,
                                                      model::RobotWrapper::FrameIndex frame_b, double angle_max)
{
  return add_constraint(new ConeConstraint(frame_a, frame_b, angle_max));
}

ConeConstraint& KinematicsSolver::add_cone_constraint(std::string frame_a, std::string frame_b, double angle_max)
{
  return add_cone_constraint(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), angle_max);
}

YawConstraint& KinematicsSolver::add_yaw_constraint(model::RobotWrapper::FrameIndex frame_a,
                                                    model::RobotWrapper::FrameIndex frame_b, double angle_max)
{
  return add_constraint(new YawConstraint(frame_a, frame_b, angle_max));
}

YawConstraint& KinematicsSolver::add_yaw_constraint(std::string frame_a, std::string frame_b, double angle_max)
{
  return add_yaw_constraint(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), angle_max);
}

DistanceConstraint& KinematicsSolver::add_distance_constraint(model::RobotWrapper::FrameIndex frame_a,
                                                              model::RobotWrapper::FrameIndex frame_b,
                                                              double distance_max)
{
  return add_constraint(new DistanceConstraint(frame_a, frame_b, distance_max));
}

DistanceConstraint& KinematicsSolver::add_distance_constraint(std::string frame_a, std::string frame_b,
                                                              double distance_max)
{
  return add_distance_constraint(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), distance_max);
}

void KinematicsSolver::mask_dof(std::string dof)
{
  masked_dof.insert(robot.get_joint_v_offset(dof));
}

void KinematicsSolver::unmask_dof(std::string dof)
{
  masked_dof.erase(robot.get_joint_v_offset(dof));
}

void KinematicsSolver::mask_fbase(bool masked)
{
  masked_fbase = masked;
}

void KinematicsSolver::enable_joint_limits(bool enable)
{
  joint_limits = enable;
}

void KinematicsSolver::enable_velocity_limits(bool enable)
{
  velocity_limits = enable;
}

int KinematicsSolver::tasks_count()
{
  return tasks.size();
}

void KinematicsSolver::compute_limits_inequalities()
{
  if (velocity_limits && dt == 0.)
  {
    throw std::runtime_error("You enabled velocity limits but didn't set solver.dt");
  }

  if (joint_limits)
  {
    problem.add_constraint(robot.state.q.bottomRows(N - 6) + qd->expr(6) <=
                           robot.model.upperPositionLimit.bottomRows(N - 6));

    problem.add_constraint(robot.model.lowerPositionLimit.bottomRows(N - 6) <=
                           robot.state.q.bottomRows(N - 6) + qd->expr(6));
  }

  if (velocity_limits)
  {
    problem.add_constraint(qd->expr(6) <= dt * robot.model.velocityLimit.bottomRows(N - 6));
    problem.add_constraint(-dt * robot.model.velocityLimit.bottomRows(N - 6) <= qd->expr(6));
  }
}

Eigen::VectorXd KinematicsSolver::solve(bool apply)
{
  // Ensure variable is created
  if (qd == nullptr)
  {
    qd = &problem.add_variable(N);
  }

  // Clear previously created constraints
  problem.clear_constraints();

  has_scaling = false;

  // Updating all the task matrices
  for (auto task : tasks)
  {
    task->update();

    // Skipping empty tasks
    if (task->A.rows() == 0)
    {
      continue;
    }

    // This could be written (task->A * qd->expr() == task->b), but would come with the
    // significant cost of multiplying A with identity matrix for each task
    Expression e;
    e.A = task->A;

    ProblemConstraint::Priority task_priority = ProblemConstraint::Hard;

    if (task->priority == Task::Priority::Scaled)
    {
      if (!has_scaling)
      {
        has_scaling = true;

        // We introduce a scale factor 0 <= s <= 1 that we want to take as close as possible to 1.0
        if (scale_variable == nullptr)
        {
          scale_variable = &problem.add_variable(1);
        }
        problem.add_constraint(scale_variable->expr() >= 0);
        problem.add_constraint(scale_variable->expr() <= 1);
        problem.add_constraint(scale_variable->expr() == 1).configure(ProblemConstraint::Soft, 1.0);
      }
      Expression scaled_error_minus_A = task->b * scale_variable->expr();
      scaled_error_minus_A.A.block(0, 0, e.A.rows(), e.A.cols()) -= e.A;
      e.A = -scaled_error_minus_A.A;

      e.b = Eigen::VectorXd::Zero(task->b.rows());
    }
    else if (task->priority == Task::Priority::Soft)
    {
      task_priority = ProblemConstraint::Soft;
      e.b = -task->b;
    }
    else
    {
      e.b = -task->b;
    }

    problem.add_constraint(e == 0).configure(task_priority, task->weight);
  }

  // Masked DoFs are hard equality constraints enforcing no deltas
  for (auto& joint : masked_dof)
  {
    problem.add_constraint(qd->expr(joint, 1) == 0);
  }

  if (masked_fbase)
  {
    problem.add_constraint(qd->expr(0, 6) == 0.);
  }

  compute_limits_inequalities();

  for (auto constraint : constraints)
  {
    constraint->add_constraint(problem);
  }

  problem.solve();

  // Retrieving qd (ignoring slack variables)
  Eigen::VectorXd qd_sol = qd->value;

  if (has_scaling)
  {
    scale = scale_variable->value(0, 0);
  }

  if (apply)
  {
    // Initial robot configuration
    auto q_save = robot.state.q;

    robot.state.q = pinocchio::integrate(robot.model, robot.state.q, qd_sol);
    if (dt > 0)
    {
      auto qd_save = robot.state.qd;
      robot.state.qd = pinocchio::difference(robot.model, q_save, robot.state.q) / dt;
      robot.state.qdd = (robot.state.qd - qd_save) / dt;
    }
  }

  return qd_sol;
}

void KinematicsSolver::clear()
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
}

std::set<Task*> KinematicsSolver::get_tasks()
{
  return tasks;
}

void KinematicsSolver::remove_task(Task& task)
{
  tasks.erase(&task);

  if (task.solver_memory)
  {
    delete &task;
  }
}

void KinematicsSolver::remove_task(FrameTask& task)
{
  tasks.erase(task.position);
  tasks.erase(task.orientation);

  if (task.position->solver_memory)
  {
    delete task.position;
  }
  if (task.orientation->solver_memory)
  {
    delete task.orientation;
  }
}

void KinematicsSolver::remove_constraint(Constraint& constraint)
{
  constraints.erase(&constraint);

  if (constraint.solver_memory)
  {
    delete &constraint;
  }
}

void KinematicsSolver::dump_status_stream(std::ostream& stream)
{
  stream << "* Kinematics Tasks:" << std::endl;
  if (has_scaling)
  {
    stream << "  * Scaling: " << scale << std::endl;
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
    else if (task->priority == Task::Priority::Scaled)
    {
      stream << "scaled";
    }
    else
    {
      stream << "soft (weight:" << task->weight << ")";
    }
    stream << std::endl;
    char buffer[128];
    sprintf(buffer, "    - Error: %.06f [%s]\n", task->error_norm(), task->error_unit().c_str());
    stream << buffer << std::endl;
  }
}

void KinematicsSolver::add_task(Task& task)
{
  task.solver = this;
  tasks.insert(&task);
}

void KinematicsSolver::add_constraint(Constraint& constraint)
{
  constraint.solver = this;
  constraints.insert(&constraint);
}

void KinematicsSolver::dump_status()
{
  dump_status_stream(std::cout);
}

};  // namespace placo::kinematics