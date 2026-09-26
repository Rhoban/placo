#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/tools/utils.h"
#include "placo/tools/prioritized.h"

namespace placo::dynamics
{
class DynamicsSolver;
class Task : public tools::Prioritized
{
public:
  /**
   * @brief Reference to the dynamics solver
   */
  DynamicsSolver* solver = nullptr;

  /**
   * @brief true if this object memory is in the solver (it will be deleted by the solver)
   */
  bool solver_memory = false;

  /**
   * @brief A matrix in Ax = b, where x is the accelerations. When \ref columns is not empty, A is compact: its k-th
   * column is the column columns[k] of the full matrix, the other columns being zero (see \ref dense_A). Else, A is
   * the full matrix.
   */
  Eigen::MatrixXd A;

  /**
   * @brief b vector in Ax = b, where x is the accelerations
   */
  Eigen::MatrixXd b;

  /**
   * @brief Degrees of freedom (sorted) the task depends on, which are the columns of \ref A (empty if A is the full
   * matrix)
   */
  std::vector<int> columns;

  /**
   * @brief Current error vector
   */
  Eigen::MatrixXd error;

  /**
   * @brief Current velocity error vector
   */
  Eigen::MatrixXd derror;

  /**
   * @brief Computes the structure of the task: the degrees of freedom it depends on (\ref columns). By default,
   * the task depends on all of them (A is the full matrix).
   */
  virtual void support();

  /**
   * @brief Fills the task matrices from the robot state and targets, A having the \ref columns computed by
   * \ref support
   */
  virtual void fill();

  /**
   * @brief Update the task matrices (\ref support, then \ref fill). Tasks can either implement \ref support and
   * \ref fill, or this method (A then being the full matrix)
   */
  virtual void update();

  /**
   * @brief The full matrix A (with all the degrees of freedom as columns)
   * @return full matrix A
   */
  Eigen::MatrixXd dense_A() const;

  /**
   * @brief Type name
   * @return string representation of the task type
   */
  virtual std::string type_name() = 0;

  /**
   * @brief Error unit
   * @return string representation of the error unit
   */
  virtual std::string error_unit() = 0;

  /**
   * @brief K gain for position control
   */
  double kp = 1e3;

  /**
   * @brief D gain for position control (if negative, will be critically damped)
   */
  double kd = -1;

  /**
   * @brief If true, the task is about tau and not about qdd
   */
  bool tau_task = false;

  /**
   * @brief Gets the kd to actually use
   * @return if critically_damped, kd will be computed from kp, otherwise kd will be returned
   */
  virtual double get_kd();

protected:
  /**
   * @brief Buffers for compact Jacobians and their time variations (their memory is reused)
   */
  Eigen::MatrixXd J_a, J_b, dJ_a, dJ_b;

  /**
   * @brief Velocities of the degrees of freedom in \ref columns (see \ref gather_qd)
   */
  Eigen::VectorXd qd_columns;

  /**
   * @brief Updates \ref qd_columns from the robot state
   */
  void gather_qd();
};
}  // namespace placo::dynamics