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
   * @brief A matrix in Ax = b, where x is the accelerations
   */
  Eigen::MatrixXd A;

  /**
   * @brief b vector in Ax = b, where x is the accelerations
   */
  Eigen::MatrixXd b;

  /**
   * @brief Current error vector
   */
  Eigen::MatrixXd error;

  /**
   * @brief Current velocity error vector
   */
  Eigen::MatrixXd derror;

  /**
   * @brief Update the task matrices
   */
  virtual void update() = 0;

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
};
}  // namespace placo::dynamics