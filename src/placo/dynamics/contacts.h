#pragma once

#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"
#include "placo/dynamics/relative_position_task.h"

namespace placo
{
namespace dynamics
{
class PositionTask;
class OrientationTask;
class DynamicsSolver;

class Contact
{
public:
  Contact();

  struct Wrench
  {
    Eigen::MatrixXd J;
    Expression f;
  };

  // Friction coefficient
  double mu = 1.;

  // Weights for optimization
  double weight_forces = 1e-6;
  double weight_moments = 1e-3;

  // Adds the wrench to the problem
  virtual Wrench add_wrench(RobotWrapper& robot, Problem& problem) = 0;

  // Wrench variable for the solver
  Variable* variable;

  DynamicsSolver* solver = nullptr;
};

class PointContact : public Contact
{
public:
  PointContact(PositionTask& position_task, bool unilateral);

  PositionTask* position_task;
  bool unilateral;

  virtual Wrench add_wrench(RobotWrapper& robot, Problem& problem);
};

class PlanarContact : public Contact
{
public:
  PlanarContact(PositionTask& position_task, OrientationTask& orientation_task, bool unilateral);

  PositionTask* position_task;
  OrientationTask* orientation_task;
  bool unilateral;

  // Length is along x axis in local frame, and width along y axis
  double length = 0.;
  double width = 0.;

  // Returns the ZMP of the contact
  Eigen::Vector3d zmp();

  virtual Wrench add_wrench(RobotWrapper& robot, Problem& problem);
};

}  // namespace dynamics
}  // namespace placo