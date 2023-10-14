#pragma once

#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"

namespace placo
{
namespace dynamics
{
class PositionTask;
class OrientationTask;
class DynamicsSolver;
struct Contact
{
  Contact();

  struct Wrench
  {
    Eigen::MatrixXd J;
    Expression f;
  };

  std::string frame_name = "";

  enum Type
  {
    Fixed = 0,
    FixedPoint = 1,
    Planar = 2,
    Point = 3
  };

  Type type = Fixed;

  // Configures the contact
  void configure(const std::string& frame_name, Type type, double mu = 1., double length = 0., double width = 0.);

  // For planar contacts, the length and width of the contact rectangle
  // Length is along x axis in local frame, and width along y axis
  double length = 0.;
  double width = 0.;

  // Friction coefficient
  double mu = 1.;

  // Weights for optimization
  double weight_forces = 1e-6;
  double weight_moments = 1e-3;

  // Adds the wrench to the problem
  Wrench add_wrench(RobotWrapper& robot, Problem& problem);

  // Returns the ZMP of the contact
  Eigen::Vector3d zmp();

  // Wrench computed by the solver
  Eigen::MatrixXd wrench;
  Variable* variable;

  DynamicsSolver* solver = nullptr;
};
}  // namespace dynamics
}  // namespace placo