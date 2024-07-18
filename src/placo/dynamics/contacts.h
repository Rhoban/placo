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

  /**
   * @brief true if the contact is active (ignored by the solver else, this allow to enable/disable
   * a contact without removing it from the solver)
   */
  bool active = true;

  /**
   * @brief true if this object memory is in the solver (it will be deleted by the solver)
   */
  bool solver_memory = false;

  /**
   * @brief Coefficient of friction (if relevant)
   */
  double mu = 1.;

  /**
   * @brief Weight of forces for the optimization (if relevant)
   */
  double weight_forces = 0.;

  /**
   * @brief Extra cost for tangential forces
   */
  double weight_tangentials = 0.;

  /**
   * @brief Weight of moments for optimization (if relevant)
   */
  double weight_moments = 0.;

  /**
   * @brief Returns the size of the contact (number of forces added to the problem)
   */
  int size();

  /**
   * @brief Contact constraint jacobian
   */
  Eigen::MatrixXd J;

  /**
   * @brief Computes the constraint jacobian
   */
  virtual void update() = 0;

  /**
   * @brief Adds contact constraints on the f expression
   * @param problem problem to which the constraints are added
   */
  virtual void add_constraints(problem::Problem& problem);

  /**
   * @brief Expression of the forces applied on the contact, created by the \ref DynamicsSolver::solve call
   */
  problem::Expression f;

  /**
   * @brief Wrench populated after the \ref DynamicsSolver::solve call
   */
  Eigen::VectorXd wrench;

  /**
   * @brief Dynamics solver associated with this contact
   */
  DynamicsSolver* solver = nullptr;
};

class PointContact : public Contact
{
public:
  /**
   * @brief see \ref DynamicsSolver::add_point_contact and \ref DynamicsSolver::add_unilateral_point_contact
   */
  PointContact(PositionTask& position_task, bool unilateral);

  /**
   * @brief associated position task
   */
  PositionTask* position_task;

  /**
   * @brief rotation matrix expressing the surface frame in the world frame (for unilateral contact)
   */
  Eigen::Matrix3d R_world_surface;

  /**
   * @brief true for unilateral contact with the ground
   */
  bool unilateral;

  virtual void update();
  virtual void add_constraints(problem::Problem& problem);
};

class Contact6D : public Contact
{
public:
  /**
   * @brief see \ref DynamicsSolver::add_fixed_planar_contact and \ref DynamicsSolver::add_unilateral_planar_contact
   */
  Contact6D(FrameTask& frame_task, bool unilateral);

  /**
   * @brief Associated position task
   */
  PositionTask* position_task;

  /**
   * @brief Associated orientation task
   */
  OrientationTask* orientation_task;

  /**
   * @brief true for unilateral contact with the ground
   */
  bool unilateral;

  /**
   * @brief Rectangular contact length along local x-axis
   */
  double length = 0.;

  /**
   * @brief Rectangular contact width along local y-axis
   */
  double width = 0.;

  /**
   * @brief Returns the contact ZMP in the local frame
   * @return zmp
   */
  Eigen::Vector3d zmp();

  virtual void update();
  virtual void add_constraints(problem::Problem& problem);
};

class LineContact : public Contact
{
public:
  /**
   * @brief see \ref DynamicsSolver::add_fixed_planar_contact and \ref DynamicsSolver::add_unilateral_planar_contact
   */
  LineContact(FrameTask& frame_task, bool unilateral);

  /**
   * @brief Associated position task
   */
  PositionTask* position_task;

  /**
   * @brief Associated orientation task
   */
  OrientationTask* orientation_task;

  /**
   * @brief true for unilateral contact with the ground
   */
  bool unilateral;

  /**
   * @brief Rectangular contact length along local x-axis
   */
  double length = 0.;

  /**
   * @brief rotation matrix expressing the surface frame in the world frame (for unilateral contact)
   */
  Eigen::Matrix3d R_world_surface;

  /**
   * @brief Returns the contact ZMP in the local frame
   * @return zmp
   */
  Eigen::Vector3d zmp();

  virtual void update();
  virtual void add_constraints(problem::Problem& problem);
};

class ExternalWrenchContact : public Contact
{
public:
  /**
   * @brief see \ref DynamicsSolver::add_external_wrench_contact
   */
  ExternalWrenchContact(model::RobotWrapper::FrameIndex frame_index,
                        pinocchio::ReferenceFrame reference = pinocchio::LOCAL_WORLD_ALIGNED);

  model::RobotWrapper::FrameIndex frame_index;
  Eigen::VectorXd w_ext = Eigen::VectorXd::Zero(6);
  pinocchio::ReferenceFrame reference;

  virtual void update();
  virtual void add_constraints(problem::Problem& problem);
};

class PuppetContact : public Contact
{
public:
  /**
   * @brief see \ref DynamicsSolver::add_puppet_contact
   */
  PuppetContact();

  virtual void update();
};

class TaskContact : public Contact
{
public:
  /**
   * @brief see \ref DynamicsSolver::add_task_contact
   */
  TaskContact(Task& task);

  Task* task;

  virtual void update();
};

}  // namespace placo::dynamics