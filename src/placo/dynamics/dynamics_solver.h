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
#include "placo/dynamics/mimic_task.h"
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
   * @brief Adds a contact to the solver
   * @param contact the contact to add
   */
  PointContact& add_point_contact(PositionTask& position_task);
  PointContact& add_unilateral_point_contact(PositionTask& position_task);
  RelativePointContact& add_relative_point_contact(RelativePositionTask& position_task);
  RelativeFixedContact& add_relative_fixed_contact(RelativeFrameTask& frame_task);
  PlanarContact& add_fixed_contact(FrameTask& frame_task);
  PlanarContact& add_planar_contact(FrameTask& frame_task);
  ExternalWrenchContact& add_external_wrench_contact(RobotWrapper::FrameIndex frame_index);
  ExternalWrenchContact& add_external_wrench_contact(std::string frame_name);
  PuppetContact& add_puppet_contact();
  TaskContact& add_task_contact(Task& task);

  /**
   * @brief Sets a DoF as passive
   * @param joint_name the DoF name
   * @param is_passive true if the DoF should be passive
   */
  void set_passive(const std::string& joint_name, bool is_passive = true, double kp = 0., double kd = 0.);

  PositionTask& add_position_task(pinocchio::FrameIndex frame_index, Eigen::Vector3d target_world);
  PositionTask& add_position_task(std::string frame_name, Eigen::Vector3d target_world);
  OrientationTask& add_orientation_task(pinocchio::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);
  OrientationTask& add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame);
  FrameTask add_frame_task(pinocchio::FrameIndex frame_index, Eigen::Affine3d T_world_frame);
  FrameTask add_frame_task(std::string frame_name, Eigen::Affine3d T_world_frame);

  RelativePositionTask& add_relative_position_task(pinocchio::FrameIndex frame_a_index,
                                                   pinocchio::FrameIndex frame_b_index, Eigen::Vector3d target);
  RelativePositionTask& add_relative_position_task(std::string frame_a_name, std::string frame_b_name,
                                                   Eigen::Vector3d target_world);

  RelativeOrientationTask& add_relative_orientation_task(pinocchio::FrameIndex frame_a_index,
                                                         pinocchio::FrameIndex frame_b_index, Eigen::Matrix3d R_a_b);
  RelativeOrientationTask& add_relative_orientation_task(std::string frame_a_name, std::string frame_b_name,
                                                         Eigen::Matrix3d R_a_b);
  RelativeFrameTask add_relative_frame_task(pinocchio::FrameIndex frame_a_index, pinocchio::FrameIndex frame_b_index,
                                            Eigen::Affine3d T_a_b);
  RelativeFrameTask add_relative_frame_task(std::string frame_a_name, std::string frame_b_name, Eigen::Affine3d T_a_b);

  CoMTask& add_com_task(Eigen::Vector3d target_world);
  JointsTask& add_joints_task();
  MimicTask& add_mimic_task();

  void set_static(bool is_static);

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
  void remove_contact(Contact& contact);

  double friction = 1e-3;
  double dt = 0.;
  int N;

  double qdd_safe = 1.;
  double xdd_safe = 1.;

  // Try to remove contact forces that can be deduces from passive joint equations
  bool optimize_contact_forces = false;

protected:
  // The problem instance is kept alive by the solver (so that variables etc. are available)
  Problem problem;

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