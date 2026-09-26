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
   * @brief Matrix A in the task Ax = b, where x are the joint delta positions. When \ref columns is not empty, A is
   * compact: its k-th column is the column columns[k] of the full matrix, the other columns being zero (see
   * \ref dense_A). Else, A is the full matrix.
   */
  Eigen::MatrixXd A;

  /**
   * @brief Vector b in the task Ax = b, where x are the joint delta positions
   */
  Eigen::MatrixXd b;

  /**
   * @brief Degrees of freedom (sorted) the task depends on, which are the columns of \ref A (empty if A is the full
   * matrix)
   */
  std::vector<int> columns;

  /**
   * @brief Computes the structure of the task: the degrees of freedom it depends on (\ref columns). By default,
   * the task depends on all of them (A is the full matrix).
   */
  virtual void support();

  /**
   * @brief Fills the task A and b matrices from the robot state and targets, A having the \ref columns computed
   * by \ref support
   */
  virtual void fill();

  /**
   * @brief Update the task A and b matrices from the robot state and targets (\ref support, then \ref fill). Tasks
   * can either implement \ref support and \ref fill, or this method (A then being the full matrix)
   */
  virtual void update();

  /**
   * @brief The full matrix A (with all the degrees of freedom as columns)
   * @return full matrix A
   */
  Eigen::MatrixXd dense_A() const;

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

protected:
  /**
   * @brief Buffers for compact Jacobians (their memory is reused)
   */
  Eigen::MatrixXd J_a, J_b;
};
}  // namespace placo::kinematics