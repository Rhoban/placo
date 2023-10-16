#include "placo/control/kinematics_solver.h"
#include "eiquadprog/eiquadprog.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "placo/model/robot_wrapper.h"
#include "placo/model/humanoid_robot.h"
#include "placo/problem/problem.h"
#include "placo/utils.h"

namespace placo
{
KinematicsSolver::KinematicsSolver(RobotWrapper& robot_) : robot(&robot_), masked_fbase(false)
{
  N = robot->model.nv;
  problem.use_sparsity = false;
}

KinematicsSolver::KinematicsSolver(RobotWrapper* robot_) : robot(robot_), masked_fbase(false)
{
  N = robot->model.nv;
  problem.use_sparsity = false;
}

PositionTask& KinematicsSolver::add_position_task(RobotWrapper::FrameIndex frame, Eigen::Vector3d target_world)
{
  return add_task(new PositionTask(frame, target_world));
}

PositionTask& KinematicsSolver::add_position_task(std::string frame, Eigen::Vector3d target_world)
{
  return add_position_task(robot->get_frame_index(frame), target_world);
}

RelativePositionTask& KinematicsSolver::add_relative_position_task(RobotWrapper::FrameIndex frame_a,
                                                                   RobotWrapper::FrameIndex frame_b,
                                                                   Eigen::Vector3d target)
{
  return add_task(new RelativePositionTask(frame_a, frame_b, target));
}

RelativePositionTask& KinematicsSolver::add_relative_position_task(std::string frame_a, std::string frame_b,
                                                                   Eigen::Vector3d target)
{
  return add_relative_position_task(robot->get_frame_index(frame_a), robot->get_frame_index(frame_b), target);
}

CoMTask& KinematicsSolver::add_com_task(Eigen::Vector3d targetCom_world)
{
  return add_task(new CoMTask(targetCom_world));
}

CoMBoundTask& KinematicsSolver::add_com_lb_task(double z_min)
{
  return add_task(new CoMBoundTask(z_min, -1));
}

CoMBoundTask& KinematicsSolver::add_com_ub_task(double z_max)
{
  return add_task(new CoMBoundTask(z_max, 1));
}

OrientationTask& KinematicsSolver::add_orientation_task(RobotWrapper::FrameIndex frame, Eigen::Matrix3d R_world_frame)
{
  return add_task(new OrientationTask(frame, R_world_frame));
}

OrientationTask& KinematicsSolver::add_orientation_task(std::string frame, Eigen::Matrix3d R_world_frame)
{
  return add_orientation_task(robot->get_frame_index(frame), R_world_frame);
}

RelativeOrientationTask& KinematicsSolver::add_relative_orientation_task(RobotWrapper::FrameIndex frame_a,
                                                                         RobotWrapper::FrameIndex frame_b,
                                                                         Eigen::Matrix3d R_a_b)
{
  return add_task(new RelativeOrientationTask(frame_a, frame_b, R_a_b));
}

RelativeOrientationTask& KinematicsSolver::add_relative_orientation_task(std::string frame_a, std::string frame_b,
                                                                         Eigen::Matrix3d R_a_b)
{
  return add_relative_orientation_task(robot->get_frame_index(frame_a), robot->get_frame_index(frame_b), R_a_b);
}

AxisAlignTask& KinematicsSolver::add_axisalign_task(RobotWrapper::FrameIndex frame, Eigen::Vector3d axis_frame,
                                                    Eigen::Vector3d targetAxis_world)
{
  return add_task(new AxisAlignTask(frame, axis_frame, targetAxis_world));
}

AxisAlignTask& KinematicsSolver::add_axisalign_task(std::string frame, Eigen::Vector3d axis_frame,
                                                    Eigen::Vector3d target_axis_world)
{
  return add_axisalign_task(robot->get_frame_index(frame), axis_frame, target_axis_world);
}

AxisPlaneTask& KinematicsSolver::add_axisplane_task(RobotWrapper::FrameIndex frame, Eigen::Vector3d axis_frame,
                                                    Eigen::Vector3d normal_world)
{
  return add_task(new AxisPlaneTask(frame, axis_frame, normal_world));
}

AxisPlaneTask& KinematicsSolver::add_axisplane_task(std::string frame, Eigen::Vector3d axis_frame,
                                                    Eigen::Vector3d normal_world)
{
  return add_axisplane_task(robot->get_frame_index(frame), axis_frame, normal_world);
}

FrameTask KinematicsSolver::add_frame_task(RobotWrapper::FrameIndex frame, Eigen::Affine3d T_world_frame)
{
  PositionTask& position = add_position_task(frame, T_world_frame.translation());
  OrientationTask& orientation = add_orientation_task(frame, T_world_frame.rotation());

  return FrameTask(&position, &orientation);
}

FrameTask KinematicsSolver::add_frame_task(std::string frame, Eigen::Affine3d T_world_frame)
{
  return add_frame_task(robot->get_frame_index(frame), T_world_frame);
}

RelativeFrameTask KinematicsSolver::add_relative_frame_task(RobotWrapper::FrameIndex frame_a,
                                                            RobotWrapper::FrameIndex frame_b, Eigen::Affine3d T_a_b)
{
  RelativePositionTask& position = add_relative_position_task(frame_a, frame_b, T_a_b.translation());
  RelativeOrientationTask& orientation = add_relative_orientation_task(frame_a, frame_b, T_a_b.rotation());

  return RelativeFrameTask(position, orientation);
}

RelativeFrameTask KinematicsSolver::add_relative_frame_task(std::string frame_a, std::string frame_b,
                                                            Eigen::Affine3d T_a_b)
{
  return add_relative_frame_task(robot->get_frame_index(frame_a), robot->get_frame_index(frame_b), T_a_b);
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

DistanceTask& KinematicsSolver::add_distance_task(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b,
                                                  double distance)
{
  return add_task(new DistanceTask(frame_a, frame_b, distance));
}

DistanceTask& KinematicsSolver::add_distance_task(std::string frame_a, std::string frame_b, double distance)
{
  return add_distance_task(robot->get_frame_index(frame_a), robot->get_frame_index(frame_b), distance);
}

CentroidalMomentumTask& KinematicsSolver::add_centroidal_momentum_task(Eigen::Vector3d L_world)
{
  return add_task(new CentroidalMomentumTask(L_world));
}

JointsTask& KinematicsSolver::add_joints_task()
{
  return add_task(new JointsTask());
}

RegularizationTask& KinematicsSolver::add_regularization_task(double magnitude)
{
  RegularizationTask& task = add_task(new RegularizationTask());
  task.set_name("regularization");
  task.set_weight(magnitude);

  return task;
}

void KinematicsSolver::mask_dof(std::string dof)
{
  masked_dof.insert(robot->get_joint_v_offset(dof));
}

void KinematicsSolver::unmask_dof(std::string dof)
{
  masked_dof.erase(robot->get_joint_v_offset(dof));
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

void KinematicsSolver::enable_velocity_post_limits(bool enable)
{
  velocity_post_limits = enable;
}

void KinematicsSolver::enable_self_collision_avoidance(bool enable, double margin, double trigger)
{
  avoid_self_collisions = enable;
  self_collisions_margin = margin;
  self_collisions_trigger = trigger;
}

void KinematicsSolver::configure_self_collision_avoidance(bool soft, double weight)
{
  self_collisions_soft = soft;
  self_collisions_weight = weight;
}

int KinematicsSolver::tasks_count()
{
  return tasks.size();
}

void KinematicsSolver::compute_self_collision_inequalities()
{
  if (avoid_self_collisions && robot != nullptr)
  {
    std::vector<RobotWrapper::Distance> distances = robot->distances();

    int constraints = 0;

    for (auto& distance : distances)
    {
      if (distance.min_distance < self_collisions_trigger)
      {
        constraints += 1;
      }
    }

    Expression e;
    e.A = Eigen::MatrixXd(constraints, N);
    e.b = Eigen::VectorXd(constraints);
    int constraint = 0;

    for (auto& distance : distances)
    {
      if (distance.min_distance < self_collisions_trigger)
      {
        Eigen::Vector3d v = distance.pointB - distance.pointA;
        Eigen::Vector3d n = v.normalized();

        if (distance.min_distance < 0)
        {
          // If the distance is negative, the points "cross" and this vector should point the other way around
          n = -n;
        }

        Eigen::MatrixXd X_A_world = pinocchio::SE3(Eigen::Matrix3d::Identity(), -distance.pointA).toActionMatrix();
        Eigen::MatrixXd JA = X_A_world * robot->joint_jacobian(distance.parentA, pinocchio::ReferenceFrame::WORLD);

        Eigen::MatrixXd X_B_world = pinocchio::SE3(Eigen::Matrix3d::Identity(), -distance.pointB).toActionMatrix();
        Eigen::MatrixXd JB = X_B_world * robot->joint_jacobian(distance.parentB, pinocchio::ReferenceFrame::WORLD);

        // We want: current_distance + J dq >= margin
        e.A.block(constraint, 0, 1, N) = n.transpose() * (JB - JA).block(0, 0, 3, N);
        e.b[constraint] = distance.min_distance - self_collisions_margin;

        constraint += 1;
      }
    }

    problem.add_constraint(e >= 0).configure(self_collisions_soft ? ProblemConstraint::Soft : ProblemConstraint::Hard,
                                             self_collisions_weight);
  }
}

void KinematicsSolver::compute_limits_inequalities()
{
  if ((velocity_limits || velocity_post_limits) && dt == 0.)
  {
    throw std::runtime_error("You enabled velocity limits but didn't set solver.dt");
  }

  int constraints = 0;
  if (joint_limits)
  {
    constraints += 2 * (N - 6);
  }
  if (velocity_limits)
  {
    constraints += 2 * (N - 6);
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
      if (joint_limits)
      {
        e.A(constraint, k + 6) = 1;
        e.b[constraint] = robot->state.q[k + 7] - robot->model.upperPositionLimit[k + 7];

        e.A(constraint + 1, k + 6) = -1;
        e.b[constraint + 1] = -robot->state.q[k + 7] + robot->model.lowerPositionLimit[k + 7];

        constraint += 2;
      }

      if (velocity_limits)
      {
        e.A(constraint, k + 6) = 1;
        e.b[constraint] = -(dt * robot->model.velocityLimit(k + 6));

        e.A(constraint + 1, k + 6) = -1;
        e.b[constraint + 1] = -(dt * robot->model.velocityLimit(k + 6));

        constraint += 2;
      }
    }

    problem.add_constraint(e <= 0);
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

  // Adding some random noise
  auto q_save = robot->state.q;

  if (noise > 0)
  {
    auto q_random = pinocchio::randomConfiguration(robot->model);

    // Adding some noise in direction of a random configuration (except floating base)
    robot->state.q.block(7, 0, robot->model.nq - 7, 1) +=
        (q_random.block(7, 0, robot->model.nq - 7, 1) - robot->state.q.block(7, 0, robot->model.nq - 7, 1)) * noise;
  }

  has_scaling = false;

  // Updating all the task matrices
  for (auto task : tasks)
  {
    task->update();

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

    if (task->equality_task)
    {
      problem.add_constraint(e == 0).configure(task_priority, task->weight);
    }
    else
    {
      problem.add_constraint(e <= 0).configure(task_priority, task->weight);
    }
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
  compute_self_collision_inequalities();

  problem.solve();

  // Retrieving qd (ignoring slack variables)
  Eigen::VectorXd qd_sol = qd->value;

  if (has_scaling)
  {
    scale = scale_variable->value(0, 0);
  }

  if (velocity_post_limits)
  {
    double ratio = 1.0;

    for (int k = 0; k < N - 6; k++)
    {
      double max_variation = dt * robot->model.velocityLimit[k + 6];
      double variation = fabs(qd_sol[k + 6]);

      if (variation > max_variation)
      {
        ratio = std::min(ratio, max_variation / variation);
      }
    }

    qd_sol = qd_sol * ratio;
  }

  if (apply)
  {
    robot->state.q = pinocchio::integrate(robot->model, robot->state.q, qd_sol);
    if (dt > 0)
    {
      robot->state.qd = pinocchio::difference(robot->model, q_save, robot->state.q) / dt;
    }
  }
  else
  {
    robot->state.q = q_save;
  }

  return qd_sol;
}

void KinematicsSolver::clear_tasks()
{
  for (auto& task : tasks)
  {
    delete task;
  }

  tasks.clear();
}

std::set<Task*> KinematicsSolver::get_tasks()
{
  return tasks;
}

void KinematicsSolver::remove_task(Task* task)
{
  tasks.erase(task);

  delete task;
}

void KinematicsSolver::remove_task(FrameTask& task)
{
  tasks.erase(task.position);
  tasks.erase(task.orientation);

  delete task.position;
  delete task.orientation;
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

void KinematicsSolver::dump_status()
{
  dump_status_stream(std::cout);
}

};  // namespace placo