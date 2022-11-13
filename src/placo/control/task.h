#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/mobile_robot.h"
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
    Soft = 1
  };

  Task();

  KinematicsSolver* solver;
  std::string name;

  void set_priority_value(Priority priority);
  void set_priority(std::string priority);
  void set_weight(double weight);
  void set_name(std::string name);

  void configure(std::string name, std::string priority = "soft", double weight = 1.0);

  // Task priority (hard: equality constraint, soft: objective function)
  Priority priority;

  // If the task is "soft", this is its weight
  double weight;

  // The task is of type Ax = b
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;

  virtual void update() = 0;
  virtual std::string type_name() = 0;
  virtual std::string error_unit() = 0;
  virtual double error();
};
}  // namespace placo