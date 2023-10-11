#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/utils.h"

namespace placo
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

  Task();
  virtual ~Task();

  KinematicsSolver* solver;
  std::string name;

  void set_priority_value(Priority priority);
  void set_priority(std::string priority);
  void set_weight(double weight);
  void set_name(std::string name);

  void configure(std::string name, std::string priority = "soft", double weight = 1.0);
  void configure(std::string name, Priority priority = Soft, double weight = 1.0);

  // Task priority (hard: equality constraint, soft: objective function)
  Priority priority;
  std::string priority_name();

  // If the task is "soft", this is its weight
  double weight;

  // The task is of type Ax = b if equality_task is true, and Ax <= b if equality_task is false
  bool equality_task = true;
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;

  virtual void update() = 0;
  virtual std::string type_name() = 0;
  virtual std::string error_unit() = 0;
  virtual Eigen::MatrixXd error();
  virtual double error_norm();
};
}  // namespace placo