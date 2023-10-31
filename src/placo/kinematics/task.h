#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/utils.h"

namespace placo::kinematics
{
class KinematicsSolver;
class Task
{
public:
  enum Priority
  {
    Hard = 0,
    Soft = 1,
    Scaled = 2
  };

  enum Type
  {
    Equality = 0,
    Inequality = 1
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

  // Task priority (hard: equality constraint, soft: objective function)
  Priority priority;
  std::string priority_name();

  // Task type (equality or inequality)
  Type type = Equality;

  // If the task is "soft", this is its weight
  double weight;

  Eigen::MatrixXd A;
  Eigen::MatrixXd b;

  virtual void update() = 0;
  virtual std::string type_name() = 0;
  virtual std::string error_unit() = 0;
  virtual Eigen::MatrixXd error();

  /**
   * @brief The task error norm
   * @return task error norm
   */
  virtual double error_norm();
};
}  // namespace placo::kinematics