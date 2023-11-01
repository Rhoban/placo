#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
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
class Task
{
public:
  /**
   * @brief Represents the task priority, which can be hard, soft or scaled
   */
  enum Priority
  {
    /**
     * @brief Task that **has** to be enforced, will result in equality constraint,
     */
    Hard = 0,
    /**
     * @brief Task that can be unenforced, will result in an objective function (can be weighted)
     */
    Soft = 1,
    /**
     * @brief This is similar to ``Hard``, but with a scaling factor (that is also a decision variable)
     */
    Scaled = 2
  };

  Task();
  virtual ~Task();

  /**
   * @brief Instance of kinematics solver
   */
  KinematicsSolver* solver;

  /**
   * @brief Task name
   */
  std::string name;

  /**
   * @brief Sets the task priority
   * @param priority Priority value (hard, soft or scaled)
   */
  void set_priority_value(Priority priority);

  /**
   * @brief Sets the task priority
   * @param priority Priority value (hard, soft or scaled)
   */
  void set_priority(std::string priority);

  /**
   * @brief Sets the task weight (for soft tasks)
   * @param weight weight value
   */
  void set_weight(double weight);
  void set_name(std::string name);

  /**
   * @brief Configures the task
   * @param name task name
   * @param priority task priority (hard, soft or scaled)
   * @param weight task weight
   */
  void configure(std::string name, std::string priority = "soft", double weight = 1.0);

  /**
   * @brief Configures the task
   * @param name task name
   * @param priority task priority (hard, soft or scaled)
   * @param weight task weight
   */
  void configure(std::string name, Priority priority = Soft, double weight = 1.0);

  /**
   * @brief Task priority
   */
  Priority priority;

  /**
   * @brief Task priority
   * @return string representing the task priority
   */
  std::string priority_name();

  /**
   * @brief The task weight, used for soft tasks only
   */
  double weight;

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