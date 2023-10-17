#pragma once

#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"

namespace placo
{
namespace dynamics
{
class PositionTask;
class RelativePositionTask;
class OrientationTask;
class DynamicsSolver;
class FrameTask;

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
  double weight_forces = 0.;
  double weight_moments = 0.;

  // Adds the wrench to the problem
  virtual Wrench add_wrench(Problem& problem) = 0;

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

  virtual Wrench add_wrench(Problem& problem);
};

class PlanarContact : public Contact
{
public:
  PlanarContact(FrameTask& frame_task, bool unilateral);

  PositionTask* position_task;
  OrientationTask* orientation_task;
  bool unilateral;

  // Length is along x axis in local frame, and width along y axis
  double length = 0.;
  double width = 0.;

  // Returns the ZMP of the contact expressed in the local frame
  Eigen::Vector3d zmp();

  virtual Wrench add_wrench(Problem& problem);
};

class RelativePointContact : public Contact
{
public:
  RelativePointContact(RelativePositionTask& position_task);

  RelativePositionTask* relative_position_task;

  virtual Wrench add_wrench(Problem& problem);
};

class ExternalWrenchContact : public Contact
{
public:
  ExternalWrenchContact(RobotWrapper::FrameIndex frame_index);

  RobotWrapper::FrameIndex frame_index;
  Eigen::VectorXd w_ext = Eigen::VectorXd::Zero(6);

  virtual Wrench add_wrench(Problem& problem);
};

class PuppetContact : public Contact
{
public:
  PuppetContact();

  virtual Wrench add_wrench(Problem& problem);
};

}  // namespace dynamics
}  // namespace placo