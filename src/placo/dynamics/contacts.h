#pragma once

#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"

namespace placo::dynamics
{
class PositionTask;
class OrientationTask;
class FrameTask;
class RelativePositionTask;
class RelativeOrientationTask;
class RelativeFrameTask;
class DynamicsSolver;
class Task;

class Contact
{
public:
  Contact();
  virtual ~Contact();

  // Friction coefficient
  double mu = 1.;

  // Weights for optimization
  double weight_forces = 0.;
  double weight_moments = 0.;

  double reaction_ratio = -1;

  int size();

  Eigen::MatrixXd J;
  virtual void update() = 0;
  virtual void add_constraints(Problem& problem);
  virtual bool is_internal();

  // Wrench variable for the solver
  Eigen::VectorXd wrench;
  Expression f;

  DynamicsSolver* solver = nullptr;
};

class PointContact : public Contact
{
public:
  PointContact(PositionTask& position_task, bool unilateral);

  PositionTask* position_task;
  bool unilateral;

  virtual void update();
  virtual void add_constraints(Problem& problem);
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

  virtual void update();
  virtual void add_constraints(Problem& problem);
};

class RelativePointContact : public Contact
{
public:
  RelativePointContact(RelativePositionTask& position_task);

  RelativePositionTask* relative_position_task;

  virtual void update();
  virtual void add_constraints(Problem& problem);
  virtual bool is_internal();
};

class RelativeFixedContact : public Contact
{
public:
  RelativeFixedContact(RelativeFrameTask& frame_task);

  RelativePositionTask* relative_position_task;
  RelativeOrientationTask* relative_orientation_task;

  virtual void update();
  virtual bool is_internal();
};

class ExternalWrenchContact : public Contact
{
public:
  ExternalWrenchContact(RobotWrapper::FrameIndex frame_index);

  RobotWrapper::FrameIndex frame_index;
  Eigen::VectorXd w_ext = Eigen::VectorXd::Zero(6);

  virtual void update();
};

class PuppetContact : public Contact
{
public:
  PuppetContact();

  virtual void update();
};

class TaskContact : public Contact
{
public:
  TaskContact(Task& task);

  Task* task;

  virtual void update();
};

}  // namespace placo::dynamics