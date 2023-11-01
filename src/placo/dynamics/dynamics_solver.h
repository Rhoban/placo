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
#include "placo/dynamics/gear_task.h"
#include "placo/dynamics/com_task.h"

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

  struct PassiveJoint
  {
    double kp;
    double kd;
  };

  DynamicsSolver(RobotWrapper& robot);
  virtual ~DynamicsSolver();

  // Contacts
  std::vector<Contact*> contacts;

  // Passive joints
  std::map<std::string, PassiveJoint> passive_joints;

  /**
   * @brief Sets the robot as static, this will impose the joint accelerations to be zero
   * @param is_static whether the robot should be static
   */
  void set_static(bool is_static);

  /**
   * @brief Sets a DoF as passive, the corresponding tau will be fixed in the equation of motion
   *        it can be purely passive joint or a spring-like behaviour
   * @param joint_name the joint
   * @param is_passive true if the should the joint be passive
   * @param kp kp gain if the joint is a spring (0 by default)
   * @param kd kd gain if the joint is a spring (0 by default)
   */
  void set_passive(const std::string& joint_name, bool is_passive = true, double kp = 0., double kd = 0.);

  /**
   * @brief Adds a position (in the world) task
   * @param frame_index target frame
   * @param target_world target position in the world
   * @return position task
   */
  PositionTask& add_position_task(RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world);

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
   */
  OrientationTask& add_orientation_task(RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);

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
   */
  FrameTask add_frame_task(RobotWrapper::FrameIndex frame_index, Eigen::Affine3d T_world_frame);

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
   */
  RelativePositionTask& add_relative_position_task(RobotWrapper::FrameIndex frame_a_index,
                                                   RobotWrapper::FrameIndex frame_b_index, Eigen::Vector3d target);

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
   */
  RelativeOrientationTask& add_relative_orientation_task(RobotWrapper::FrameIndex frame_a_index,
                                                         RobotWrapper::FrameIndex frame_b_index, Eigen::Matrix3d R_a_b);

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
   */
  RelativeFrameTask add_relative_frame_task(RobotWrapper::FrameIndex frame_a_index,
                                            RobotWrapper::FrameIndex frame_b_index, Eigen::Affine3d T_a_b);

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
   * @param target target joints values
   * @return joints task
   */
  JointsTask& add_joints_task();

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
  ExternalWrenchContact& add_external_wrench_contact(RobotWrapper::FrameIndex frame_index);

  /**
   * @brief Adds an external wrench
   * @param frame_index
   * @return external wrench contact
   */
  ExternalWrenchContact& add_external_wrench_contact(std::string frame_name);

  /**
   * @brief Adds a puppet contact, this will add some free contact forces for the whole system, allowing it
   *        to be controlled freely
   */
  PuppetContact& add_puppet_contact();

  /**
   * @brief Adds contact forces associated with any given task
   */
  TaskContact& add_task_contact(Task& task);

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
   * @brief Enables or disable the self collision inequalities
   * @param enable whether to enable the self collision inequalities
   * @param margin margin that will be used [m]
   * @param trigger the trigger distance at which the inequalities are enabled [m]
   */
  void enable_self_collision_avoidance(bool enable, double margin = 0.005, double trigger = 0.01);

  /**
   * @brief Changes the self collision configuration
   */
  void configure_self_collision_avoidance(bool soft, double weight);

  /**
   * @brief Enables/disables torque limits inequalities
   */
  void enable_torque_limits(bool enable);

  void compute_reaction_ratio_inequalities();
  void compute_limits_inequalities(Expression& tau);
  void compute_self_collision_inequalities();

  /**
   * @brief Clears the internal tasks
   */
  void clear_tasks();

  /**
   * @brief Dumps the status to a given stream
   */
  void dump_status_stream(std::ostream& stream);

  /**
   * @brief Shows the tasks status
   */
  void dump_status();

  Result solve();

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

  RobotWrapper& robot;

  void remove_task(Task& task);
  void remove_task(FrameTask& task);
  void remove_contact(Contact& contact);

  double friction = 1e-3;
  double dt = 0.;
  int N;

  double qdd_safe = 1.;

  // Try to remove contact forces that can be deduces from passive joint equations
  bool optimize_contact_forces = false;

  // The problem instance is kept alive by the solver (so that variables etc. are available)
  Problem problem;

protected:
  // Masked DoFs (enforce zero acceleration)
  std::set<int> masked_dof;
  bool masked_fbase;

  // Tasks
  std::set<Task*> tasks;

  // Task id (this is only useful when task names are not specified, each task will have an unique ID)
  int task_id = 0;

  // Limits
  bool torque_limits = false;
  bool joint_limits = false;
  bool velocity_vs_torque_limits = false;
  bool velocity_limits = false;

  // Self collision prevention
  bool avoid_self_collisions = false;
  double self_collisions_margin = 0.005;  // [m]
  double self_collisions_trigger = 0.01;  // [m]

  // Self collisions configuration
  bool self_collisions_soft = false;
  double self_collisions_weight = 1.;

  // If true, the solver will assume qdd = 0
  bool is_static = false;

  template <typename T>
  T& add_task(T* task)
  {
    task_id += 1;
    task->solver = this;
    std::ostringstream oss;
    oss << "Task_" << task_id;
    task->name = oss.str();
    tasks.insert(task);

    return *task;
  }

  template <typename T>
  T& add_contact(T* contact)
  {
    contact->solver = this;
    contacts.push_back(contact);

    return *contact;
  }
};
}  // namespace placo::dynamics