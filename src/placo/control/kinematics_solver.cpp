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

KinematicsSolver::Equality::Equality(Eigen::MatrixXd A, Eigen::VectorXd b) : A(A), b(b)
{
}

KinematicsSolver::Objective::Objective(Eigen::MatrixXd P, Eigen::VectorXd q, double weight) : P(P), q(q), weight(weight)
{
}

KinematicsSolver::KinematicsSolver(MobileRobot& robot) : robot(robot)
{
  N = robot.model.nv;
}

void KinematicsSolver::add_position_task(MobileRobot::FrameIndex frame, Eigen::Vector3d target_world, Priority priority,
                                         double weight)
{
  auto T_world_frame = robot.get_T_world_frame(frame);
  Eigen::Vector3d error = target_world - T_world_frame.translation();
  auto J = robot.frame_jacobian(frame, pinocchio::LOCAL_WORLD_ALIGNED);

  create_task(J.block(0, 0, 3, N), error, priority, weight);
}

void KinematicsSolver::add_position_task(std::string frame, Eigen::Vector3d target_world, std::string priority,
                                         double weight)
{
  add_position_task(robot.get_frame_index(frame), target_world, priority_from_string(priority), weight);
}

void KinematicsSolver::add_com_task(Eigen::Vector3d targetCom_world, Priority priority, double weight)
{
  Eigen::Vector3d error = targetCom_world - robot.com_world();

  create_task(robot.com_jacobian(), error, priority, weight);
}

void KinematicsSolver::add_com_task(Eigen::Vector3d targetCom_world, std::string priority, double weight)
{
  add_com_task(targetCom_world, priority_from_string(priority), weight);
}

void KinematicsSolver::add_orientation_task(MobileRobot::FrameIndex frame, Eigen::Matrix3d R_world_target,
                                            Priority priority, double weight)
{
  auto T_world_frame = robot.get_T_world_frame(frame);

  // (R_target R_current^{-1}) R_current = R_target
  // |-----------------------|
  //            | This part is the world error that "correct" the rotation
  //            matrix to the desired one
  Eigen::Vector3d error = pinocchio::log3(R_world_target * T_world_frame.linear().inverse());

  auto J = robot.frame_jacobian(frame, pinocchio::WORLD);

  create_task(J.block(3, 0, 3, N), error, priority, weight);
}

void KinematicsSolver::add_orientation_task(std::string frame, Eigen::Matrix3d R_world_target, std::string priority,
                                            double weight)
{
  add_orientation_task(robot.get_frame_index(frame), R_world_target, priority_from_string(priority), weight);
}

void KinematicsSolver::add_frame_task(MobileRobot::FrameIndex frame, Eigen::Affine3d frame_target, Priority priority,
                                      double position_weight, double orientation_weight)
{
  add_position_task(frame, frame_target.translation(), priority, position_weight);
  add_orientation_task(frame, frame_target.rotation(), priority, orientation_weight);
}

void KinematicsSolver::add_frame_task(std::string frame, Eigen::Affine3d T_world_target, std::string priority,
                                      double position_weight, double orientation_weight)
{
  add_frame_task(robot.get_frame_index(frame), T_world_target, priority_from_string(priority), position_weight,
                 orientation_weight);
}

void KinematicsSolver::add_pose_task(MobileRobot::FrameIndex frame, Eigen::Affine3d T_world_target, Priority priority,
                                     double weight)
{
  auto T_world_frame = robot.get_T_world_frame(frame);

  // (T_target T_current^{-1}) T_current = T_target
  // |-----------------------|
  //            | This part is the world error that "correct" the transformatio
  Eigen::VectorXd error = pinocchio::log6((T_world_target * T_world_frame.inverse()).matrix()).toVector();

  auto J = robot.frame_jacobian(frame, pinocchio::WORLD);

  create_task(J, error, priority, weight);
}

void KinematicsSolver::add_pose_task(std::string frame, Eigen::Affine3d T_world_target, std::string priority,
                                     double weight)
{
  add_pose_task(robot.get_frame_index(frame), T_world_target, priority_from_string(priority), weight);
}

void KinematicsSolver::add_regularization_task(double magnitude)
{
  Eigen::MatrixXd P = Eigen::MatrixXd(N, N);
  Eigen::VectorXd q = Eigen::VectorXd(N);

  P.setIdentity();
  q.setZero();

  objectives.push_back(Objective(P, q, magnitude));
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

  // Building objective function
  Eigen::MatrixXd P(N, N);
  Eigen::VectorXd q(N);

  P.setZero();
  q.setZero();

  for (auto& objective : objectives)
  {
    P += objective.weight * objective.P;
    q += objective.weight * objective.q;
  }

  // Building equality constraint by stacking all the matrices
  int n_equalities = 0;

  // Counting equality constraints

  // Masked DoFs
  n_equalities += masked_dof.size();

  // Constraint matrices
  for (auto& equality : equalities)
  {
    n_equalities += equality.A.rows();
  }
  Eigen::MatrixXd A(n_equalities, N);
  Eigen::VectorXd b(n_equalities);
  A.setZero();
  b.setZero();

  // Adding constraints
  int k = 0;
  for (auto& equality : equalities)
  {
    A.block(k, 0, equality.A.rows(), N) = equality.A;
    b.block(k, 0, equality.A.rows(), 1) = equality.b;
    k += equality.A.rows();
  }

  // Adding 1s in columns of a, yielding delta_q_i = 0 and preventing the DoF from being
  // used by the solver
  for (auto& joint : masked_dof)
  {
    A(k, joint) = 1;
    k += 1;
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

  // Clearing current equalities and objectives
  equalities.clear();
  objectives.clear();

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

void KinematicsSolver::create_task(Eigen::MatrixXd A, Eigen::VectorXd b, Priority priority, double weight)
{
  if (priority == Soft)
  {
    // Ax = b
    // minimizes (Ax - b)^T (Ax - b)
    // Thus, (x^T A^T - b^T) (Ax - b) = x^T A^T A x - 2 b^T A
    // P (Hessian) is A^T A
    // q (linear) is - b^T A
    // We removed the "2" because the solver solves for 1/2 x^T P x + q^T x
    objectives.push_back(Objective(A.transpose() * A, -b.transpose() * A, weight));
  }
  else
  {
    equalities.push_back(Equality(A, b));
  }
}

};  // namespace placo