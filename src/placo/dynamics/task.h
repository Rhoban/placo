#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/tools/utils.h"

namespace placo::dynamics
{
class DynamicsSolver;
class Task
{
public:
  enum Priority
  {
    Hard = 0,
    Soft = 1
  };

  Task();
  virtual ~Task();

  DynamicsSolver* solver;
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

  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
  Eigen::MatrixXd error;
  Eigen::MatrixXd derror;

  virtual void update() = 0;
  virtual std::string type_name() = 0;
  virtual std::string error_unit() = 0;

  // Gains for PD control
  double kp = 1e3;
  double kd = 0.;
  bool critically_damped = true;

  virtual double get_kd();
};
}  // namespace placo::dynamics