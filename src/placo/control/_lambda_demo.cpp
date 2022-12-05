#include "placo/control/kinematics_solver.h"
#include "eiquadprog/eiquadprog.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "placo/model/robot_wrapper.h"
#include "placo/utils.h"

namespace placo
{
KinematicsSolver::KinematicsSolver(RobotWrapper* robot_) : robot(robot_), masked_fbase(false)
{
  N = robot->model.nv;
}

KinematicsSolver::KinematicsSolver(RobotWrapper& robot_) : robot(&robot_), masked_fbase(false)
{
  N = robot->model.nv;
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

  return FrameTask(position, orientation);
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

PoseTask& KinematicsSolver::add_pose_task(RobotWrapper::FrameIndex frame, Eigen::Affine3d T_world_frame)
{
  return add_task(new PoseTask(frame, T_world_frame));
}

PoseTask& KinematicsSolver::add_pose_task(std::string frame, Eigen::Affine3d T_world_frame)
{
  return add_pose_task(robot->get_frame_index(frame), T_world_frame);
}

RelativePoseTask& KinematicsSolver::add_relative_pose_task(RobotWrapper::FrameIndex frame_a,
                                                           RobotWrapper::FrameIndex frame_b, Eigen::Affine3d T_a_b)
{
  return add_task(new RelativePoseTask(frame_a, frame_b, T_a_b));
}

RelativePoseTask& KinematicsSolver::add_relative_pose_task(std::string frame_a, std::string frame_b,
                                                           Eigen::Affine3d T_a_b)
{
  return add_relative_pose_task(robot->get_frame_index(frame_a), robot->get_frame_index(frame_b), T_a_b);
}

JointTask& KinematicsSolver::add_joint_task(std::string joint, double target)
{
  return add_task(new JointTask(joint, target));
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

void KinematicsSolver::configure_limits(bool dofs_limit_, bool speeds_limit_, bool speed_post_limits_)
{
  dofs_limit = dofs_limit_;
  speeds_limit = speeds_limit_;
  speed_post_limits = speed_post_limits_;
}

Eigen::VectorXd KinematicsSolver::solve(bool apply)
{
  // Adding some random noise
  auto q_save = robot->state.q;

  if (noise > 0)
  {
    auto q_random = pinocchio::randomConfiguration(robot->model);

    // Adding some noise in direction of a random configuration (except floating base)
    robot->state.q.block(7, 0, robot->model.nq - 7, 1) +=
        (q_random.block(7, 0, robot->model.nq - 7, 1) - robot->state.q.block(7, 0, robot->model.nq - 7, 1)) * noise;
  }

  Eigen::VectorXd qd;

  // Updating all the task matrices
  for (auto task : tasks)
  {
    task->update();
  }

  // Building objective function
  Eigen::MatrixXd P(N + 1, N + 1);
  Eigen::VectorXd q(N + 1);

  P.setZero();
  q.setZero();

  int n_equalities = 0;

  for (auto task : tasks)
  {
    if (task->priority == Task::Priority::Soft)
    {
      // The original equality is: Ax = b
      // In term of minimization, we want to minimize (Ax - b)^T (Ax - b)
      // Thus, (x^T A^T - b^T) (Ax - b) = x^T A^T A x - 2 b^T A
      // P (hessian) is A^T A
      // q (linear) is - b^T A
      // We removed the "2" because the solver already solves for 1/2 x^T P x + q^T x
      Eigen::MatrixXd A_l(task->A.rows(), N + 1);
      A_l.block(0, 0, task->A.rows(), task->A.cols()) = task->A;
      A_l.block(0, N, task->A.rows(), 1) = task->b;

      P += task->weight * (A_l.transpose() * A_l);
      q += task->weight * (-task->b.transpose() * A_l);
    }
    else
    {
      n_equalities += task->A.rows();
    }
  }

  P(N, N) += 1e-6;

  // Adding masked DoF in equality constraints
  n_equalities += masked_dof.size();

  if (masked_fbase)
  {
    n_equalities += 6;
  }

  // Constraint matrices
  Eigen::MatrixXd A(n_equalities, N + 1);
  Eigen::VectorXd b(n_equalities);
  A.setZero();
  b.setZero();

  // Adding constraints
  int current_equality_row = 0;

  for (auto task : tasks)
  {
    if (task->priority == Task::Priority::Hard)
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

  if (masked_fbase)
  {
    // Masking the 6 first dof to disable fbase in optimization
    for (int k = 0; k < 6; k++)
    {
      A(current_equality_row, k) = 1;
      current_equality_row += 1;
    }
  }

  // Handling degree of freedoms limit inequalities
  int constrained_dofs = N - 6;
  int n_inequalities = 2;
  if (dofs_limit)
  {
    n_inequalities += constrained_dofs * 2;
  }
  if (speeds_limit)
  {
    n_inequalities += constrained_dofs * 2;
  }
  Eigen::MatrixXd CI(n_inequalities, N + 1);
  Eigen::VectorXd ci0(n_inequalities);
  CI.setZero();
  ci0.setZero();
  int current_inequality_row = 0;

  for (int k = 0; k < N - 6; k++)
  {
    Eigen::MatrixXd selector(1, N);
    selector.setZero();
    selector(0, k + 6) = 1;

    if (dofs_limit)
    {
      // Position limits
      CI.block(current_inequality_row, 0, 1, N) = selector;
      CI.block(current_inequality_row + 1, 0, 1, N) = -selector;

      ci0[current_inequality_row] = -(robot->model.lowerPositionLimit[k + 7] - robot->state.q[k + 7]);
      ci0[current_inequality_row + 1] = (robot->model.upperPositionLimit[k + 7] - robot->state.q[k + 7]);

      current_inequality_row += 2;
    }

    if (speeds_limit)
    {
      // Speed limits
      CI.block(current_inequality_row, 0, 1, N) = selector;
      CI.block(current_inequality_row + 1, 0, 1, N) = -selector;

      ci0[current_inequality_row] = dt * robot->model.velocityLimit[k + 6];
      ci0[current_inequality_row + 1] = dt * robot->model.velocityLimit[k + 6];

      current_inequality_row += 2;
    }
  }

  CI(current_inequality_row, N) = 1;  // l >= 0
  ci0[current_inequality_row] = 0;

  CI(current_inequality_row + 1, N) = -1;  // l <= 1
  ci0[current_inequality_row + 1] = 1;

  Eigen::VectorXi activeSet;
  size_t activeSetSize;
  double result =
      eiquadprog::solvers::solve_quadprog(P, q, A.transpose(), -b, CI.transpose(), ci0, qd, activeSet, activeSetSize);

  if (result == std::numeric_limits<double>::infinity())
  {
    throw std::runtime_error("KinematicsSolver: Infeasible QP (check your equality and inequality constraints)");
  }

  if (qd.hasNaN())
  {
    throw std::runtime_error("KinematicsSolver: NaN encountered in result");
  }

  std::cout << "Lambda=" << qd(N, 0) << std::endl;

  if (speed_post_limits)
  {
    double ratio = 1.0;

    for (int k = 0; k < N - 6; k++)
    {
      double max_variation = dt * robot->model.velocityLimit[k + 6];
      double variation = fabs(qd[k + 6]);

      if (variation > max_variation)
      {
        ratio = std::min(ratio, max_variation / variation);
      }
    }

    qd = qd * ratio;
  }

  if (apply)
  {
    robot->state.q = pinocchio::integrate(robot->model, robot->state.q, qd.block(0, 0, N, 1));
  }
  else
  {
    robot->state.q = q_save;
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
    if (task->priority == Task::Priority::Hard)
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