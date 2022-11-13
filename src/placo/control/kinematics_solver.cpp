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

KinematicsSolver::RelativePositionTask::RelativePositionTask(MobileRobot::FrameIndex frame_a,
                                                             MobileRobot::FrameIndex frame_b, Eigen::Vector3d target)
  : frame_a(frame_a), frame_b(frame_b), target(target)
{
}

KinematicsSolver::CoMTask::CoMTask(Eigen::Vector3d target_world) : target_world(target_world)
{
}

KinematicsSolver::OrientationTask::OrientationTask(MobileRobot::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
  : frame_index(frame_index), R_world_frame(R_world_frame)
{
}

KinematicsSolver::RelativeOrientationTask::RelativeOrientationTask(MobileRobot::FrameIndex frame_a,
                                                                   MobileRobot::FrameIndex frame_b,
                                                                   Eigen::Matrix3d R_a_b)
  : frame_a(frame_a), frame_b(frame_b), R_a_b(R_a_b)
{
}

KinematicsSolver::FrameTask::FrameTask(PositionTask& position, OrientationTask& orientation)
  : position(position), orientation(orientation)
{
}

KinematicsSolver::RelativeFrameTask::RelativeFrameTask(RelativePositionTask& position,
                                                       RelativeOrientationTask& orientation)
  : position(position), orientation(orientation)
{
}

void KinematicsSolver::FrameTask::configure(std::string name, std::string priority, double position_weight,
                                            double orientation_weight)
{
  position.configure(name + "_position", priority, position_weight);
  orientation.configure(name + "_orientation", priority, orientation_weight);
}

Eigen::Affine3d KinematicsSolver::FrameTask::get_T_world_frame() const
{
  Eigen::Affine3d T;
  T.translation() = position.target_world;
  T.linear() = orientation.R_world_frame;
  return T;
}

void KinematicsSolver::FrameTask::set_T_world_frame(Eigen::Affine3d T_world_frame)
{
  position.target_world = T_world_frame.translation();
  orientation.R_world_frame = T_world_frame.linear();
}

void KinematicsSolver::RelativeFrameTask::configure(std::string name, std::string priority, double position_weight,
                                                    double orientation_weight)
{
  position.configure(name + "_position", priority, position_weight);
  orientation.configure(name + "_orientation", priority, orientation_weight);
}

Eigen::Affine3d KinematicsSolver::RelativeFrameTask::get_T_a_b() const
{
  Eigen::Affine3d T;
  T.translation() = position.target;
  T.linear() = orientation.R_a_b;
  return T;
}

void KinematicsSolver::RelativeFrameTask::set_T_a_b(Eigen::Affine3d T_a_b)
{
  position.target = T_a_b.translation();
  orientation.R_a_b = T_a_b.linear();
}

KinematicsSolver::AxisAlignTask::AxisAlignTask(MobileRobot::FrameIndex frame_index, Eigen::Vector3d axis_frame,
                                               Eigen::Vector3d targetAxis_world)
  : frame_index(frame_index), axis_frame(axis_frame), targetAxis_world(targetAxis_world)
{
}

KinematicsSolver::PoseTask::PoseTask(MobileRobot::FrameIndex frame_index, Eigen::Affine3d T_world_frame)
  : frame_index(frame_index), T_world_frame(T_world_frame)
{
}

KinematicsSolver::RelativePoseTask::RelativePoseTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b,
                                                     Eigen::Affine3d T_a_b)
  : frame_a(frame_a), frame_b(frame_b), T_a_b(T_a_b)
{
}

KinematicsSolver::JointTask::JointTask(std::string joint, double target) : joint(joint), target(target)
{
}

KinematicsSolver::JointsTask::JointsTask()
{
}

void KinematicsSolver::JointsTask::set_joint(std::string joint, double target)
{
  joints[joint] = target;
}

void KinematicsSolver::PositionTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  Eigen::Vector3d error = target_world - T_world_frame.translation();
  auto J = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL_WORLD_ALIGNED);

  A = J.block(0, 0, 3, solver->N);
  b = error;
}

std::string KinematicsSolver::PositionTask::type_name()
{
  return "position";
}

std::string KinematicsSolver::PositionTask::error_unit()
{
  return "m";
}

void KinematicsSolver::RelativePositionTask::update()
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  Eigen::Vector3d error = target - T_a_b.translation();

  auto J_a = solver->robot.frame_jacobian(frame_a, pinocchio::WORLD);
  auto J_b = solver->robot.frame_jacobian(frame_b, pinocchio::WORLD);

  A = (pinocchio::SE3(T_world_a.inverse().matrix()).toActionMatrix() * (J_b - J_a)).block(0, 0, 3, solver->N);
  b = error;
}

std::string KinematicsSolver::RelativePositionTask::type_name()
{
  return "relative_position";
}

std::string KinematicsSolver::RelativePositionTask::error_unit()
{
  return "m";
}

void KinematicsSolver::CoMTask::update()
{
  A = solver->robot.com_jacobian();
  b = target_world;
}

std::string KinematicsSolver::CoMTask::type_name()
{
  return "com";
}

std::string KinematicsSolver::CoMTask::error_unit()
{
  return "m";
}

void KinematicsSolver::OrientationTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);

  // (R_frame R_current^{-1}) R_current = R_frame
  // |-----------------------|
  //            | This part is the world error that "correct" the rotation
  //            matrix to the desired one
  Eigen::Vector3d error = pinocchio::log3(R_world_frame * T_world_frame.linear().inverse());

  auto J = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD);

  A = J.block(3, 0, 3, solver->N);
  b = error;
}

std::string KinematicsSolver::OrientationTask::type_name()
{
  return "orientation";
}

std::string KinematicsSolver::OrientationTask::error_unit()
{
  return "rad";
}

void KinematicsSolver::RelativeOrientationTask::update()
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  // (R_a_b* R_a_b^{-1}) R_a_b = R_a_b*
  // |-----------------|
  //         | This part is the world error that "correct" the rotation
  //           matrix to the desired one
  Eigen::Vector3d error = pinocchio::log3(R_a_b * T_a_b.linear().inverse());

  auto J_a = solver->robot.frame_jacobian(frame_a, pinocchio::WORLD);
  auto J_b = solver->robot.frame_jacobian(frame_b, pinocchio::WORLD);
  Eigen::MatrixXd J_ab = pinocchio::SE3(T_world_a.inverse().matrix()).toActionMatrix() * (J_b - J_a);

  A = J_ab.block(3, 0, 3, solver->N);
  b = error;
}

std::string KinematicsSolver::RelativeOrientationTask::type_name()
{
  return "relative_orientation";
}

std::string KinematicsSolver::RelativeOrientationTask::error_unit()
{
  return "rad";
}

void KinematicsSolver::AxisAlignTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  auto targetAxis_world_normalized = targetAxis_world.normalized();

  // Here, we will define an "axis frame", which x axis is aligned with the current axis, the z axis is the axis
  // we need to rotate about to correct the error, and y the last axis
  // Thus, expressing the Jacobian in this frame, we can let the rotation about the x axis free, and control
  // the rotation about z to be the error, and about y to be zero.
  Eigen::Matrix3d R_world_axisframe;
  R_world_axisframe.col(0) = (T_world_frame.rotation() * axis_frame).normalized();
  R_world_axisframe.col(2) = R_world_axisframe.col(0).cross(targetAxis_world_normalized).normalized();
  R_world_axisframe.col(1) = R_world_axisframe.col(2).cross(R_world_axisframe.col(0));

  // Computing the error angle we want to compensate
  double error_angle = acos(R_world_axisframe.col(0).dot(targetAxis_world_normalized));

  // We express the Jacobian in the axisframe
  Eigen::MatrixXd J_axisframe = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD).block(3, 0, 3, solver->N);
  J_axisframe = (R_world_axisframe.inverse() * J_axisframe);

  // We only keep y and z in the constraint, since we don't care about rotations about x axis in the axis frame
  A = J_axisframe.block(1, 0, 2, solver->N);
  b = Eigen::Vector2d(0., error_angle);
}

std::string KinematicsSolver::AxisAlignTask::type_name()
{
  return "axis_align";
}

std::string KinematicsSolver::AxisAlignTask::error_unit()
{
  return "rad";
}

void KinematicsSolver::PoseTask::update()
{
  auto T_world_frame_current = solver->robot.get_T_world_frame(frame_index);

  // (T_frame T_current^{-1}) T_current = T_frame
  // |-----------------------|
  //            | This part is the world error that "correct" the transformatio
  Eigen::VectorXd error = pinocchio::log6((T_world_frame * T_world_frame_current.inverse()).matrix()).toVector();

  A = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD);
  b = error;
}

void KinematicsSolver::RelativePoseTask::update()
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);
  auto T_a_b_current = T_world_a.inverse() * T_world_b;

  // (T_a_b* T_a_b^{-1}) T_a_b = T_a_b*
  // |-----------------|
  //          | This part is the world error that "correct" the transformatio
  Eigen::VectorXd error = pinocchio::log6((T_a_b * T_a_b_current.inverse()).matrix()).toVector();

  auto J_a = solver->robot.frame_jacobian(frame_a, pinocchio::WORLD);
  auto J_b = solver->robot.frame_jacobian(frame_b, pinocchio::WORLD);
  A = pinocchio::SE3(T_world_a.inverse().matrix()).toActionMatrix() * (J_b - J_a);
  b = error;
}

std::string KinematicsSolver::PoseTask::type_name()
{
  return "pose";
}

std::string KinematicsSolver::PoseTask::error_unit()
{
  return "twist-norm";
}

std::string KinematicsSolver::RelativePoseTask::type_name()
{
  return "relative_pose";
}

std::string KinematicsSolver::RelativePoseTask::error_unit()
{
  return "twist-norm";
}

void KinematicsSolver::JointTask::update()
{
  A = Eigen::MatrixXd(1, solver->N);
  A.setZero();
  A(0, solver->robot.get_joint_v_offset(joint)) = 1;

  b = Eigen::MatrixXd(1, 1);
  b(0, 0) = target - solver->robot.get_joint(joint);
}

std::string KinematicsSolver::JointTask::type_name()
{
  return "joint";
}

std::string KinematicsSolver::JointTask::error_unit()
{
  return "dof-rad";
}

void KinematicsSolver::JointsTask::update()
{
  A = Eigen::MatrixXd(joints.size(), solver->N);
  b = Eigen::MatrixXd(joints.size(), 1);
  A.setZero();

  int k = 0;
  for (auto& entry : joints)
  {
    A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
    b(k, 0) = entry.second - solver->robot.get_joint(entry.first);

    k += 1;
  }
}

std::string KinematicsSolver::JointsTask::type_name()
{
  return "joints";
}

std::string KinematicsSolver::JointsTask::error_unit()
{
  return "dof-rads";
}

void KinematicsSolver::RegularizationTask::update()
{
  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  A = Eigen::MatrixXd(solver->N, solver->N);
  A.setIdentity();

  b = Eigen::MatrixXd(solver->N, 1);
  b.setZero();
}

std::string KinematicsSolver::RegularizationTask::type_name()
{
  return "regularization";
}

std::string KinematicsSolver::RegularizationTask::error_unit()
{
  return "none";
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

KinematicsSolver::RelativePositionTask& KinematicsSolver::add_relative_position_task(MobileRobot::FrameIndex frame_a,
                                                                                     MobileRobot::FrameIndex frame_b,
                                                                                     Eigen::Vector3d target)
{
  return add_task(new RelativePositionTask(frame_a, frame_b, target));
}

KinematicsSolver::RelativePositionTask& KinematicsSolver::add_relative_position_task(std::string frame_a,
                                                                                     std::string frame_b,
                                                                                     Eigen::Vector3d target)
{
  return add_relative_position_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), target);
}

KinematicsSolver::CoMTask& KinematicsSolver::add_com_task(Eigen::Vector3d targetCom_world)
{
  return add_task(new CoMTask(targetCom_world));
}

KinematicsSolver::OrientationTask& KinematicsSolver::add_orientation_task(MobileRobot::FrameIndex frame,
                                                                          Eigen::Matrix3d R_world_frame)
{
  return add_task(new OrientationTask(frame, R_world_frame));
}

KinematicsSolver::OrientationTask& KinematicsSolver::add_orientation_task(std::string frame,
                                                                          Eigen::Matrix3d R_world_frame)
{
  return add_orientation_task(robot.get_frame_index(frame), R_world_frame);
}

KinematicsSolver::RelativeOrientationTask& KinematicsSolver::add_relative_orientation_task(
    MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Matrix3d R_a_b)
{
  return add_task(new RelativeOrientationTask(frame_a, frame_b, R_a_b));
}

KinematicsSolver::RelativeOrientationTask& KinematicsSolver::add_relative_orientation_task(std::string frame_a,
                                                                                           std::string frame_b,
                                                                                           Eigen::Matrix3d R_a_b)
{
  return add_relative_orientation_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), R_a_b);
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
                                                             Eigen::Affine3d T_world_frame)
{
  PositionTask& position = add_position_task(frame, T_world_frame.translation());
  OrientationTask& orientation = add_orientation_task(frame, T_world_frame.rotation());

  return FrameTask(position, orientation);
}

KinematicsSolver::FrameTask KinematicsSolver::add_frame_task(std::string frame, Eigen::Affine3d T_world_frame)
{
  return add_frame_task(robot.get_frame_index(frame), T_world_frame);
}

KinematicsSolver::RelativeFrameTask KinematicsSolver::add_relative_frame_task(MobileRobot::FrameIndex frame_a,
                                                                              MobileRobot::FrameIndex frame_b,
                                                                              Eigen::Affine3d T_a_b)
{
  RelativePositionTask& position = add_relative_position_task(frame_a, frame_b, T_a_b.translation());
  RelativeOrientationTask& orientation = add_relative_orientation_task(frame_a, frame_b, T_a_b.rotation());

  return RelativeFrameTask(position, orientation);
}

KinematicsSolver::RelativeFrameTask KinematicsSolver::add_relative_frame_task(std::string frame_a, std::string frame_b,
                                                                              Eigen::Affine3d T_a_b)
{
  return add_relative_frame_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), T_a_b);
}

KinematicsSolver::PoseTask& KinematicsSolver::add_pose_task(MobileRobot::FrameIndex frame,
                                                            Eigen::Affine3d T_world_frame)
{
  return add_task(new PoseTask(frame, T_world_frame));
}

KinematicsSolver::PoseTask& KinematicsSolver::add_pose_task(std::string frame, Eigen::Affine3d T_world_frame)
{
  return add_pose_task(robot.get_frame_index(frame), T_world_frame);
}

KinematicsSolver::RelativePoseTask& KinematicsSolver::add_relative_pose_task(MobileRobot::FrameIndex frame_a,
                                                                             MobileRobot::FrameIndex frame_b,
                                                                             Eigen::Affine3d T_a_b)
{
  return add_task(new RelativePoseTask(frame_a, frame_b, T_a_b));
}

KinematicsSolver::RelativePoseTask& KinematicsSolver::add_relative_pose_task(std::string frame_a, std::string frame_b,
                                                                             Eigen::Affine3d T_a_b)
{
  return add_relative_pose_task(robot.get_frame_index(frame_a), robot.get_frame_index(frame_b), T_a_b);
}

KinematicsSolver::JointTask& KinematicsSolver::add_joint_task(std::string joint, double target)
{
  return add_task(new JointTask(joint, target));
}

KinematicsSolver::JointsTask& KinematicsSolver::add_joints_task(std::map<std::string, double>& joints)
{
  KinematicsSolver::JointsTask& task = add_task(new JointsTask());
  for (auto& entry : joints)
  {
    task.joints[entry.first] = entry.second;
  }
  return task;
}

KinematicsSolver::JointsTask& KinematicsSolver::add_joints_task()
{
  return add_task(new JointsTask());
}

KinematicsSolver::RegularizationTask& KinematicsSolver::add_regularization_task(double magnitude)
{
  KinematicsSolver::RegularizationTask& task = add_task(new RegularizationTask());
  task.set_name("regularization");
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
    throw std::runtime_error("KinematicsSolver: Infeasible QP (check your equality and inequality constraints)");
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

void KinematicsSolver::dump_status()
{
  std::cout << "* Kinematics Tasks:" << std::endl;
  for (auto task : tasks)
  {
    task->update();
    std::cout << "  * " << task->name << " [" << task->type_name() << "]" << std::endl;
    std::cout << "    - Priority: ";
    if (task->priority == Hard)
    {
      std::cout << "hard";
    }
    else
    {
      std::cout << "soft (weight:" << task->weight << ")";
    }
    std::cout << std::endl;
    printf("    - Error: %.06f [%s]\n", task->error(), task->error_unit().c_str());
    std::cout << std::endl;
  }
}

};  // namespace placo