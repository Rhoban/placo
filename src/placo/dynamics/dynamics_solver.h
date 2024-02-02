#pragma once

#include <Eigen/Dense>
#include <set>
#include <map>

// Tasks
#include "placo/model/robot_wrapper.h"
#include "placo/tools/axises_mask.h"
#include "placo/dynamics/contacts.h"
#include "placo/dynamics/task.h"
#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"
#include "placo/dynamics/frame_task.h"
#include "placo/dynamics/relative_position_task.h"
#include "placo/dynamics/relative_orientation_task.h"
#include "placo/dynamics/relative_frame_task.h"
#include "placo/dynamics/joints_task.h"
#include "placo/dynamics/torque_task.h"
#include "placo/dynamics/gear_task.h"
#include "placo/dynamics/com_task.h"

// Constraints
#include "placo/dynamics/constraint.h"
#include "placo/dynamics/avoid_self_collisions_constraint.h"
#include "placo/dynamics/reaction_ratio_constraint.h"

// Problem formulation
#include "placo/problem/problem.h"

namespace placo::dynamics
{
class DynamicsSolver
{
public:
  struct Result
  {
    // Checks if the gravity computation is a success
    bool success;

    // Torques computed by the solver
    Eigen::VectorXd tau;

    // Accelerations computed by the solver
    Eigen::VectorXd qdd;
  };

  struct OverrideJoint
  {
    // If passive is true, tau will be computed as a function of kp and kd
    bool passive;
    double kp;
    double kd;

    // Else, a custom tau will be used
    double tau;
  };

  DynamicsSolver(model::RobotWrapper& robot);
  virtual ~DynamicsSolver();

  // Contacts
  std::vector<Contact*> contacts;

  // Override joints (passive or custom tau)
  std::map<std::string, OverrideJoint> override_joints;

  /**
   * @brief Sets a DoF as passive, the corresponding tau will be fixed in the equation of motion
   *        it can be purely passive joint or a spring-like behaviour
   * @param joint_name the joint
   * @param is_passive true if the should the joint be passive
   * @param kp kp gain if the joint is a spring (0 by default)
   * @param kd kd gain if the joint is a spring (0 by default)
   */
  void set_passive(const std::string& joint_name, double kp = 0., double kd = 0.);

  /**
   * @brief Sets a custom torque to be applied by a given joint
   * @param joint_name the joint
   * @param tau torque
   */
  void set_tau(const std::string& joint_name, double tau);

  /**
   * @brief Resets a given joint so that its torque is no longer overriden
   * @param joint_name the joint
   */
  void reset_joint(const std::string& joint_name);

  /**
   * @brief Adds a position (in the world) task
   * @param frame_index target frame
   * @param target_world target position in the world
   * @return position task
   * @pyignore
   */
  PositionTask& add_position_task(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world);

  /**
   * @brief Adds a position (in the world) task
   * @param frame_index target frame
   * @param target_world target position in the world
   * @return position task
   */
  PositionTask& add_position_task(std::string frame_name, Eigen::Vector3d target_world);

  /**
   * @brief Adds an orientation (in the world) task
   * @param frame_index target frame
   * @param R_world_frame target world orientation
   * @return orientation task
   * @pyignore
   */
  OrientationTask& add_orientation_task(model::RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);

  /**
   * @brief Adds an orientation (in the world) task
   * @param frame_index target frame
   * @param R_world_frame target world orientation
   * @return orientation task
   */
  OrientationTask& add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame);

  /**
   * @brief Adds a frame task, which is a pseudo-task packaging position and orientation, resulting in a
   *        decoupled task
   * @param frame_index target frame
   * @param T_world_frame target transformation in the world
   * @return frame task
   * @pyignore
   */
  FrameTask add_frame_task(model::RobotWrapper::FrameIndex frame_index, Eigen::Affine3d T_world_frame);

  /**
   * @brief Adds a frame task, which is a pseudo-task packaging position and orientation, resulting in a
   *        decoupled task
   * @param frame_index target frame
   * @param T_world_frame target transformation in the world
   * @return frame task
   */
  FrameTask add_frame_task(std::string frame_name, Eigen::Affine3d T_world_frame);

  /**
   * @brief Adds a relative position task
   * @param frame_a_index frame a
   * @param frame_b_index  frame b
   * @param target target value for AB vector, expressed in A
   * @return relative position task
   * @pyignore
   */
  RelativePositionTask& add_relative_position_task(model::RobotWrapper::FrameIndex frame_a_index,
                                                   model::RobotWrapper::FrameIndex frame_b_index,
                                                   Eigen::Vector3d target);

  /**
   * @brief Adds a relative position task
   * @param frame_a_index frame a
   * @param frame_b_index  frame b
   * @param target target value for AB vector, expressed in A
   * @return relative position task
   */
  RelativePositionTask& add_relative_position_task(std::string frame_a_name, std::string frame_b_name,
                                                   Eigen::Vector3d target_world);

  /**
   * @brief Adds a relative orientation task
   * @param frame_a_index frame a
   * @param frame_b_index frame b
   * @param R_a_b target value for the orientation of b frame in a
   * @return relative orientation task
   * @pyignore
   */
  RelativeOrientationTask& add_relative_orientation_task(model::RobotWrapper::FrameIndex frame_a_index,
                                                         model::RobotWrapper::FrameIndex frame_b_index,
                                                         Eigen::Matrix3d R_a_b);

  /**
   * @brief Adds a relative orientation task
   * @param frame_a_index frame a
   * @param frame_b_index frame b
   * @param R_a_b target value for the orientation of b frame in a
   * @return relative orientation task
   */
  RelativeOrientationTask& add_relative_orientation_task(std::string frame_a_name, std::string frame_b_name,
                                                         Eigen::Matrix3d R_a_b);

  /**
   * @brief Adds a relative frame task, which is a pseudo-task packaging relative position and orientation tasks
   * @param frame_a_index frame a
   * @param frame_b_index frame b
   * @param T_a_b target transformation value for b frame in a
   * @return relative frame task
   * @pyignore
   */
  RelativeFrameTask add_relative_frame_task(model::RobotWrapper::FrameIndex frame_a_index,
                                            model::RobotWrapper::FrameIndex frame_b_index, Eigen::Affine3d T_a_b);

  /**
   * @brief Adds a relative frame task, which is a pseudo-task packaging relative position and orientation tasks
   * @param frame_a_index frame a
   * @param frame_b_index frame b
   * @param T_a_b target transformation value for b frame in a
   * @return relative frame task
   */
  RelativeFrameTask add_relative_frame_task(std::string frame_a_name, std::string frame_b_name, Eigen::Affine3d T_a_b);

  /**
   * @brief Adds a center of mass (in the world) task
   * @param target_world target (in the world)
   * @return center of mass task
   */
  CoMTask& add_com_task(Eigen::Vector3d target_world);

  /**
   * @brief Adds a joints task
   * @return joints task
   */
  JointsTask& add_joints_task();

  /**
   * @brief Adds a torque task
   * @return torque task
   */
  TorqueTask& add_torque_task();

  /**
   * @brief Adds a gear task, allowing replication of a joint. This can be used to implement timing belt, if coupled
   *        with an internal force.
   * @return gear task
   */
  GearTask& add_gear_task();

  /**
   * @brief Adds a point contact
   * @param position_task the associated position task
   * @return point contact
   */
  PointContact& add_point_contact(PositionTask& position_task);

  /**
   * @brief Adds an unilateral point contact, in the sense of the world z-axis
   * @param position_task the associated position task
   * @return unilateral point contact
   */
  PointContact& add_unilateral_point_contact(PositionTask& position_task);

  /**
   * @brief Adds a relative point contact, which can be typically used for internal forces like loop-closing
   * @param position_task associated relative position task
   * @return relative point contact
   */
  RelativePointContact& add_relative_point_contact(RelativePositionTask& position_task);

  /**
   * @brief Adds a relative fixed contact, can be used for fixed closed loops
   * @param frame_task the associated relative frame task
   * @return relative fixed contact
   */
  Relative6DContact& add_relative_fixed_contact(RelativeFrameTask& frame_task);

  /**
   * @brief Adds a fixed contact
   * @param frame_task the associated frame task
   * @return fixed contact
   */
  Contact6D& add_fixed_contact(FrameTask& frame_task);

  /**
   * @brief Adds a planar contact, which is unilateral in the sense of the local body z-axis
   * @param frame_task associated frame task
   * @return planar contact
   */
  Contact6D& add_planar_contact(FrameTask& frame_task);

  /**
   * @brief Adds an external wrench
   * @param frame_index
   * @return external wrench contact
   */
  ExternalWrenchContact& add_external_wrench_contact(model::RobotWrapper::FrameIndex frame_index);

  /**
   * @brief Adds an external wrench
   * @param frame_name frame
   * @return external wrench contact
   */
  ExternalWrenchContact& add_external_wrench_contact(std::string frame_name);

  /**
   * @brief Adds a puppet contact, this will add some free contact forces for the whole system, allowing it
   *        to be controlled freely
   * @return puppet contact
   */
  PuppetContact& add_puppet_contact();

  /**
   * @brief Adds contact forces associated with any given task
   * @param task task
   * @return task contact
   */
  TaskContact& add_task_contact(Task& task);

  /**
   * @brief Adds a constraint to the solver
   * @return constraint
   */
  AvoidSelfCollisionsConstraint& add_avoid_self_collisions_constraint();

  /**
   * @brief Adds a constraint enforce reaction ratio
   * @param contact contact
   * @param reaction_ratio reaction ratio to enforce
   * @return reaction ratio constraint
   */
  ReactionRatioConstraint& add_reaction_ratio_constraint(Contact& contact, double reaction_ratio);

  /**
   * @brief Enables/disables joint limits inequalities
   */
  void enable_joint_limits(bool enable);

  /**
   * @brief Enables/disables joint velocity inequalities
   */
  void enable_velocity_limits(bool enable);

  /**
   * @brief Enables the velocity vs torque inequalities
   */
  void enable_velocity_vs_torque_limits(bool enable);

  /**
   * @brief Enables/disables torque limits inequalities
   */
  void enable_torque_limits(bool enable);

  /**
   * @brief Computes the joint limits inequalities
   * @param tau the torque expression
   */
  void compute_limits_inequalities(problem::Expression& tau);

  /**
   * @brief Clears the internal tasks
   */
  void clear();

  /**
   * @brief Dumps the status to a given stream
   */
  void dump_status_stream(std::ostream& stream);

  /**
   * @brief Shows the tasks status
   */
  void dump_status();

  Result solve(bool integrate = false);

  /**
   * @brief Masks (disables a DoF) from being used by the QP solver (it can't provide speed)
   * @param dof the dof name
   */
  void mask_dof(std::string dof);

  /**
   * @brief Unmsks (enables a DoF) from being used by the QP solver (it can provide speed)
   * @param dof the dof name
   */
  void unmask_dof(std::string dof);

  /**
   * @brief Decides if the floating base should be masked
   */
  void mask_fbase(bool masked);

  model::RobotWrapper& robot;

  /**
   * @brief Removes a task from the solver
   * @param task task
   */
  void remove_task(Task& task);

  /**
   * @brief Removes a frame task from the solver
   * @param task frame task
   */
  void remove_task(FrameTask& task);

  /**
   * @brief Removes a contact from the solver
   * @param contact
   */
  void remove_contact(Contact& contact);

  /**
   * @brief Removes a constraint from the solver
   * @param constraint constraint
   */
  void remove_constraint(Constraint& constraint);

  /**
   * @brief Global friction that is added to all the joints
   */
  double friction = 0.;

  /**
   * @brief Solver dt (seconds)
   */
  double dt = 0.;

  /**
   * @brief Number of variables (size of qd and qdd)
   */
  int N;

  /**
   * @brief The value of qdd safe
   */
  double qdd_safe = 1.;

  /**
   * @brief Use gravity only (no coriolis, no centrifugal)
   */
  bool gravity_only = false;

  /**
   * @brief Instance of the problem
   */
  problem::Problem problem;

  /**
   * @brief Adds a custom task to the solver
   * @param task task
   */
  void add_task(Task& task);

  /**
   * @brief Adds a custom constraint to the solver
   * @param constraint constraint
   */
  void add_constraint(Constraint& constraint);

  /**
   * @brief Adds a custom contact to the solver
   * @param contact contact
   */
  void add_contact(Contact& contact);

  /**
   * @brief Adds a task to the solver
   * @param task task
   * @return reference to internal task
   */
  template <typename T>
  T& add_task(T* task)
  {
    task_id += 1;
    task->solver = this;
    task->solver_memory = true;
    std::ostringstream oss;
    oss << "Task_" << task_id;
    task->name = oss.str();
    tasks.insert(task);

    return *task;
  }

  /**
   * @brief Adds a constraint to the solver
   * @param constraint constraint
   * @return reference to internal constraint
   */
  template <typename T>
  T& add_constraint(T* constraint)
  {
    constraint_id += 1;
    constraint->solver = this;
    constraint->solver_memory = true;
    std::ostringstream oss;
    oss << "Constraint_" << constraint_id;
    constraint->name = oss.str();
    constraints.insert(constraint);

    return *constraint;
  }

  /**
   * @brief  Adds a contact to the solver
   * @param contact contact
   * @return reference to internal contact
   */
  template <typename T>
  T& add_contact(T* contact)
  {
    contact->solver = this;
    contact->solver_memory = true;
    contacts.push_back(contact);

    return *contact;
  }

protected:
  // Disables floating base
  bool masked_fbase;

  // Tasks
  std::set<Task*> tasks;

  // Constraints
  std::set<Constraint*> constraints;

  // Task id (this is only useful when task names are not specified, each task will have an unique ID)
  int task_id = 0;
  int constraint_id = 0;

  // Limits
  bool torque_limits = false;
  bool joint_limits = false;
  bool velocity_vs_torque_limits = false;
  bool velocity_limits = false;
};
}  // namespace placo::dynamics