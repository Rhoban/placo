#pragma once

#include "placo/model/mobile_robot.h"
#include <Eigen/Dense>

namespace placo
{
class KinematicsSolver
{
public:
  KinematicsSolver(MobileRobot& robot);

  enum Priority
  {
    Hard = 0,
    Soft = 1
  };

  struct Equality
  {
    Equality(Eigen::MatrixXd A, Eigen::VectorXd b);

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
  };

  struct Objective
  {
    Objective(Eigen::MatrixXd P, Eigen::VectorXd q, double weight);

    Eigen::MatrixXd P;  // Hessian part
    Eigen::VectorXd q;  // Linear part

    double weight;
  };

  /**
   * @brief Adds a position task
   * @param frame the robot frame we want to control
   * @param target_world the target position, expressed in the world (as
   * T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective
   * function)
   * @param weight task weight (if soft)
   */
  void add_position_task(MobileRobot::FrameIndex frame, Eigen::Vector3d target_world, Priority priority = Soft,
                         double weight = 1.0);
  void add_position_task(std::string frame, Eigen::Vector3d target_world, std::string priority = "soft",
                         double weight = 1.0);

  /**
   * @brief Adds a com position task
   * @param targetCom_world the target position, expressed in the world (as
   * T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective
   * function)
   * @param weight task weight (if soft)
   */
  void add_com_task(Eigen::Vector3d targetCom_world, Priority priority = Soft, double weight = 1.0);
  void add_com_task(Eigen::Vector3d targetCom_world, std::string priority = "soft", double weight = 1.0);

  /**
   * @brief Adds an orientation task
   * @param frame the robot frame we want to control
   * @param R_world_target the target orientation we want to achieve, expressed
   * in the world (as T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective
   * function)
   * @param weight task weight (if soft)
   */
  void add_orientation_task(MobileRobot::FrameIndex frame, Eigen::Matrix3d R_world_target, Priority priority = Soft,
                            double weight = 1.0);
  void add_orientation_task(std::string frame, Eigen::Matrix3d R_world_target, std::string priority = "soft",
                            double weight = 1.0);

  /**
   * @brief Adds a frame task, this is equivalent to a position + orientation
   * task, resulting in a "decoupled" style control of a given frame
   * @param frame the robot frame we want to control
   * @param T_world_target the target for the frame we want to control,
   * expressed in the world (as T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective
   * function)
   * @param position_weight position task weight (if soft)
   * @param orientation_weight orientation task weight (if soft)
   */
  void add_frame_task(MobileRobot::FrameIndex frame, Eigen::Affine3d T_world_target, Priority priority = Soft,
                      double position_weight = 1.0, double orientation_weight = 1.0);
  void add_frame_task(std::string frame, Eigen::Affine3d T_world_target, std::string priority = "soft",
                      double position_weight = 1.0, double orientation_weight = 1.0);

  /**
   * @brief Adds a pose task. The difference with the frame task is that the
   * error will be computed in a coupled way (using the log of the matrix
   * error), resulting in a "screw" target motion. This should be used instead
   * of add_frame_task when the frame is not attached to a particular point we
   * want to control, but to a body. As a result, the position and orientation
   * tasks can't be weighted independently when this task is soft.
   * @param frame the robot frame we want to control
   * @param frame the robot frame we want to control
   * @param T_world_target the target for the frame we want to control,
   * expressed in the world (as T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective
   * function)
   * @param weight task weight (if soft)
   */
  void add_pose_task(MobileRobot::FrameIndex frame, Eigen::Affine3d T_world_target, Priority priority = Soft,
                     double weight = 1.0);
  void add_pose_task(std::string frame, Eigen::Affine3d T_world_target, std::string priority = "soft",
                     double weight = 1.0);

  /**
   * @brief Adds a regularization task for a given magnitude
   * @param magnitude regularization magnitude
   */
  void add_regularization_task(double magnitude = 1e-6);

  /**
   * @brief Constructs the QP problem and solves it
   * @return the vector containing delta q, which are target variations for the
   * robot degrees of freedom.
   */
  Eigen::VectorXd solve(bool apply = false);

  /**
   * @brief Masks (disables a DoF) from being used by the QP solver (it can't provide speed)
   * @param dof the dof name
   */
  void mask_dof(std::string dof);

  /**
   * @brief Unmsks (enables a DoF) from being used by the QP solver (it can provide speed)
   * @param dof the dof name
   */
  void unmask_dof(std::string dof);

protected:
  MobileRobot& robot;
  std::vector<Equality> equalities;
  std::vector<Objective> objectives;
  std::set<int> masked_dof;

  /**
   * @brief Creates either an equality constraint or an objective task depending
   * on the given priority
   * @param A A from Ax = b
   * @param b b from Ax = b
   * @param priority task priority (hard: equality constraint, soft: objective
   * function)
   * @param weight task weight (if soft)
   */
  void create_task(Eigen::MatrixXd A, Eigen::VectorXd b, Priority priority, double weight);

  /**
   * @brief Size of the problem (number of variables we will search)
   */
  int N;
};
}  // namespace placo