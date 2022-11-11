#include "placo/control/kinematics_solver.h"
#include "eiquadprog/eiquadprog.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace placo
{
KinematicsSolver::Priority priority_from_string(std::string priority)
{
  if (priority == "soft")
  {
    return KinematicsSolver::Priority::Soft;
  }
  else if (priority == "hard")
  {
    return KinematicsSolver::Priority::Hard;
  }
  else
  {
    throw std::runtime_error(std::string("KinematicsSolver: Invalid priority: ") + priority);
  }
}

KinematicsSolver::Task::Task()
{
  solver = nullptr;
  priority = Soft;
  weight = 1.0;
}

void KinematicsSolver::Task::set_priority_value(Priority priority_)
{
  priority = priority_;
}

void KinematicsSolver::Task::set_priority(std::string priority)
{
  priority = priority_from_string(priority);
}

void KinematicsSolver::Task::set_weight(double weight_)
{
  weight = weight_;
}

void KinematicsSolver::Task::set_name(std::string name_)
{
  name = name_;
}

void KinematicsSolver::Task::configure(std::string name_, std::string priority_, double weight_)
{
  name = name_;
  priority = priority_from_string(priority_);
  weight = weight_;
}

double KinematicsSolver::Task::error()
{
  return b.norm();
}

KinematicsSolver::PositionTask::PositionTask(MobileRobot::FrameIndex frame_index, Eigen::Vector3d target_world)
  : frame_index(frame_index), target_world(target_world)
{
}

KinematicsSolver::CoMTask::CoMTask(Eigen::Vector3d target_world) : target_world(target_world)
{
}

KinematicsSolver::OrientationTask::OrientationTask(MobileRobot::FrameIndex frame_index, Eigen::Matrix3d R_world_target)
  : frame_index(frame_index), R_world_target(R_world_target)
{
}

KinematicsSolver::FrameTask::FrameTask(PositionTask& position, OrientationTask& orientation)
  : position(position), orientation(orientation)
{
}

void KinematicsSolver::FrameTask::configure(std::string name, std::string priority, double position_weight,
                                            double orientation_weight)
{
  position.configure(name + "_position", priority, position_weight);
  orientation.configure(name + "_orientation", priority, orientation_weight);
}

KinematicsSolver::AxisAlignTask::AxisAlignTask(MobileRobot::FrameIndex frame_index, Eigen::Vector3d axis_frame,
                                               Eigen::Vector3d targetAxis_world)
  : frame_index(frame_index), axis_frame(axis_frame), targetAxis_world(targetAxis_world)
{
}

KinematicsSolver::PoseTask::PoseTask(MobileRobot::FrameIndex frame_index, Eigen::Affine3d T_world_target)
  : frame_index(frame_index), T_world_target(T_world_target)
{
}

KinematicsSolver::JointTask::JointTask(std::string joint, double target) : joint(joint), target(target)
{
}

void KinematicsSolver::PositionTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  Eigen::Vector3d error = target_world - T_world_frame.translation();
  auto J = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL_WORLD_ALIGNED);

  A = J.block(0, 0, 3, solver->N);
  b = error;
}

void KinematicsSolver::CoMTask::update()
{
  A = solver->robot.com_jacobian();
  b = target_world;
}

void KinematicsSolver::OrientationTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);

  // (R_target R_current^{-1}) R_current = R_target
  // |-----------------------|
  //            | This part is the world error that "correct" the rotation
  //            matrix to the desired one
  Eigen::Vector3d error = pinocchio::log3(R_world_target * T_world_frame.linear().inverse());

  auto J = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD);

  A = J.block(3, 0, 3, solver->N);
  b = error;
}

void KinematicsSolver::AxisAlignTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  auto target_axis_world_normalized = targetAxis_world.normalized();

  // Here, we will define an "axis frame", which x axis is aligned with the current axis, the z axis is the axis
  // we need to rotate about to correct the error, and y the last axis
  // Thus, expressing the Jacobian in this frame, we can let the rotation about the x axis free, and control
  // the rotation about z to be the error, and about y to be zero.
  Eigen::Matrix3d R_world_axisframe;
  R_world_axisframe.col(0) = (T_world_frame.rotation() * axis_frame).normalized();
  R_world_axisframe.col(2) = R_world_axisframe.col(0).cross(target_axis_world_normalized);
  R_world_axisframe.col(1) = R_world_axisframe.col(2).cross(R_world_axisframe.col(0));

  // Computing the error angle we want to compensate
  double error_angle = acos(R_world_axisframe.col(0).dot(target_axis_world_normalized));

  // We express the Jacobian in the axisframe
  auto J_axisframe = R_world_axisframe.inverse() *
                     solver->robot.frame_jacobian(frame_index, pinocchio::WORLD).block(3, 0, 3, solver->N);

  // We only keep y and z in the constraint, since we don't care about rotations about x axis in the axis frame
  A = J_axisframe.block(1, 0, 2, solver->N);
  b = Eigen::Vector2d(0., error_angle);
}

void KinematicsSolver::PoseTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);

  // (T_target T_current^{-1}) T_current = T_target
  // |-----------------------|
  //            | This part is the world error that "correct" the transformatio
  Eigen::VectorXd error = pinocchio::log6((T_world_target * T_world_frame.inverse()).matrix()).toVector();

  A = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD);
  b = error;
}

void KinematicsSolver::JointTask::update()
{
  A = Eigen::MatrixXd(1, solver->N);
  A.setZero();
  A(0, solver->robot.get_joint_v_offset(joint)) = 1;

  double delta = target - solver->robot.get_joint(joint);
  b = Eigen::MatrixXd(1, 1);
  b << delta;
}

void KinematicsSolver::RegularizationTask::update()
{
  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  A = Eigen::MatrixXd(solver->N, solver->N);
  A.setIdentity();

  b = Eigen::MatrixXd(solver->N, 1);
  b.setZero();
}

KinematicsSolver::KinematicsSolver(MobileRobot& robot) : robot(robot)
{
  N = robot.model.nv;
}

KinematicsSolver::PositionTask& KinematicsSolver::add_position_task(MobileRobot::FrameIndex frame,
                                                                    Eigen::Vector3d target_world)
{
  return add_task(new PositionTask(frame, target_world));
}

KinematicsSolver::PositionTask& KinematicsSolver::add_position_task(std::string frame, Eigen::Vector3d target_world)
{
  return add_position_task(robot.get_frame_index(frame), target_world);
}

KinematicsSolver::CoMTask& KinematicsSolver::add_com_task(Eigen::Vector3d targetCom_world)
{
  return add_task(new CoMTask(targetCom_world));
}

KinematicsSolver::OrientationTask& KinematicsSolver::add_orientation_task(MobileRobot::FrameIndex frame,
                                                                          Eigen::Matrix3d R_world_target)
{
  return add_task(new OrientationTask(frame, R_world_target));
}

KinematicsSolver::OrientationTask& KinematicsSolver::add_orientation_task(std::string frame,
                                                                          Eigen::Matrix3d R_world_target)
{
  return add_orientation_task(robot.get_frame_index(frame), R_world_target);
}

KinematicsSolver::AxisAlignTask& KinematicsSolver::add_axisalign_task(MobileRobot::FrameIndex frame,
                                                                      Eigen::Vector3d axis_frame,
                                                                      Eigen::Vector3d targetAxis_world)
{
  return add_task(new AxisAlignTask(frame, axis_frame, targetAxis_world));
}

KinematicsSolver::AxisAlignTask& KinematicsSolver::add_axisalign_task(std::string frame, Eigen::Vector3d axis_frame,
                                                                      Eigen::Vector3d target_axis_world)
{
  return add_axisalign_task(robot.get_frame_index(frame), axis_frame, target_axis_world);
}

KinematicsSolver::FrameTask KinematicsSolver::add_frame_task(MobileRobot::FrameIndex frame,
                                                             Eigen::Affine3d frame_target)
{
  PositionTask& position = add_position_task(frame, frame_target.translation());
  OrientationTask& orientation = add_orientation_task(frame, frame_target.rotation());

  return FrameTask(position, orientation);
}

KinematicsSolver::FrameTask KinematicsSolver::add_frame_task(std::string frame, Eigen::Affine3d T_world_target)
{
  return add_frame_task(robot.get_frame_index(frame), T_world_target);
}

KinematicsSolver::PoseTask& KinematicsSolver::add_pose_task(MobileRobot::FrameIndex frame,
                                                            Eigen::Affine3d T_world_target)
{
  return add_task(new PoseTask(frame, T_world_target));
}

KinematicsSolver::PoseTask& KinematicsSolver::add_pose_task(std::string frame, Eigen::Affine3d T_world_target)
{
  return add_pose_task(robot.get_frame_index(frame), T_world_target);
}

KinematicsSolver::JointTask& KinematicsSolver::add_joint_task(std::string joint, double target)
{
  return add_task(new JointTask(joint, target));
}

KinematicsSolver::RegularizationTask& KinematicsSolver::add_regularization_task(double magnitude)
{
  KinematicsSolver::RegularizationTask& task = add_task(new RegularizationTask());
  task.set_weight(magnitude);

  return task;
}

void KinematicsSolver::mask_dof(std::string dof)
{
  masked_dof.insert(robot.get_joint_v_offset(dof));
}

void KinematicsSolver::unmask_dof(std::string dof)
{
  masked_dof.erase(robot.get_joint_v_offset(dof));
}

Eigen::VectorXd KinematicsSolver::solve(bool apply)
{
  Eigen::VectorXd qd;

  // Updating all the task matrices
  for (auto task : tasks)
  {
    task->update();
  }

  // Building objective function
  Eigen::MatrixXd P(N, N);
  Eigen::VectorXd q(N);

  P.setZero();
  q.setZero();

  int n_equalities = 0;

  for (auto task : tasks)
  {
    if (task->priority == Soft)
    {
      // The original equality is: Ax = b
      // In term of minimization, we want to minimize (Ax - b)^T (Ax - b)
      // Thus, (x^T A^T - b^T) (Ax - b) = x^T A^T A x - 2 b^T A
      // P (hessian) is A^T A
      // q (linear) is - b^T A
      // We removed the "2" because the solver already solves for 1/2 x^T P x + q^T x

      P += task->weight * (task->A.transpose() * task->A);
      q += task->weight * (-task->b.transpose() * task->A);
    }
    else
    {
      n_equalities += task->A.rows();
    }
  }

  // Adding masked DoF in equality constraints
  n_equalities += masked_dof.size();

  // Constraint matrices
  Eigen::MatrixXd A(n_equalities, N);
  Eigen::VectorXd b(n_equalities);
  A.setZero();
  b.setZero();

  // Adding constraints
  int current_equality_row = 0;

  for (auto task : tasks)
  {
    if (task->priority == Hard)
    {
      A.block(current_equality_row, 0, task->A.rows(), N) = task->A;
      b.block(current_equality_row, 0, task->A.rows(), 1) = task->b;
      current_equality_row += task->A.rows();
    }
  }

  // Adding 1s in columns of a, yielding delta_q_i = 0 and preventing the DoF from being
  // used by the solver
  for (auto& joint : masked_dof)
  {
    A(current_equality_row, joint) = 1;
    current_equality_row += 1;
  }

  // Handling degree of freedoms limit inequalities
  int constrained_dofs = N - 6;
  Eigen::MatrixXd CI(2 * constrained_dofs, N);
  Eigen::VectorXd ci0(2 * constrained_dofs);

  for (int k = 0; k < N - 6; k++)
  {
    Eigen::MatrixXd selector(1, N);
    selector.setZero();
    selector(0, k + 6) = 1;
    CI.block(k * 2, 0, 1, N) = selector;
    CI.block(k * 2 + 1, 0, 1, N) = -selector;

    ci0[k * 2] = -(robot.model.lowerPositionLimit[k + 7] - robot.state.q[k + 7]);
    ci0[k * 2 + 1] = (robot.model.upperPositionLimit[k + 7] - robot.state.q[k + 7]);
  }

  Eigen::VectorXi activeSet;
  size_t activeSetSize;
  double result =
      eiquadprog::solvers::solve_quadprog(P, q, A.transpose(), -b, CI.transpose(), ci0, qd, activeSet, activeSetSize);

  if (result == std::numeric_limits<double>::infinity())
  {
    throw std::runtime_error("KinematicsSolver: Infeasible QP (check your "
                             "equality and inequality constraints)");
  }

  if (apply)
  {
    robot.state.q = pinocchio::integrate(robot.model, robot.state.q, qd);
  }

  return qd;
}

void KinematicsSolver::clear_tasks()
{
  for (auto& task : tasks)
  {
    delete task;
  }

  tasks.clear();
}

};  // namespace placo