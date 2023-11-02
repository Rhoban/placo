#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/tools/prioritized.h"
#include "placo/tools/utils.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief Represents a task for the kinematics solver.
 *
 * A task is essentially a constraint of the form \f$ Ax = b \f$, where x is the vector of joint
 * delta positions solved by the kinematics solver.
 *
 * The task can be either an equality constraint (hard task) or an objective (soft task).
 *
 * See \ref placo::kinematics::KinematicsSolver
 */
class Task : public tools::Prioritized
{
public:
  /**
   * @brief Instance of kinematics solver
   */
  KinematicsSolver* solver = nullptr;

  /**
   * @brief true if this object memory is in the solver (it will be deleted by the solver)
   */
  bool solver_memory = false;

  /**
   * @brief Matrix A in the task Ax = b, where x are the joint delta positions
   */
  Eigen::MatrixXd A;

  /**
   * @brief Vector b in the task Ax = b, where x are the joint delta positions
   */
  Eigen::MatrixXd b;

  /**
   * @brief Update the task A and b matrices from the robot state and targets
   */
  virtual void update() = 0;

  /**
   * @brief Name of the task type
   * @return string representing the task type
   */
  virtual std::string type_name() = 0;

  /**
   * @brief Unit of the task error
   * @return string representing the task error unit
   */
  virtual std::string error_unit() = 0;

  /**
   * @brief Task errors (vector)
   * @return task errors
   */
  virtual Eigen::MatrixXd error();

  /**
   * @brief The task error norm
   * @return task error norm
   */
  virtual double error_norm();
};
}  // namespace placo::kinematics