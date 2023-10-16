#pragma once

#include <Eigen/Dense>
#include <set>
#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"
#include "placo/control/axises_mask.h"
#include "placo/dynamics/contacts.h"
#include "placo/dynamics/task.h"
#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"
#include "placo/dynamics/frame_task.h"
#include "placo/dynamics/relative_position_task.h"
#include "placo/dynamics/joints_task.h"
#include "placo/dynamics/static_task.h"
#include "placo/dynamics/com_task.h"

namespace placo
{
namespace dynamics
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

  struct LoopClosure
  {
    std::string frame_a;
    std::string frame_b;
    AxisesMask mask;
  };

  DynamicsSolver(RobotWrapper& robot);
  virtual ~DynamicsSolver();

  // Contacts
  std::vector<Contact*> contacts;

  // Passive joints
  std::set<std::string> passive_joints;

  // Desired accelerations
  Eigen::VectorXd qdd_desired = Eigen::VectorXd();

  /**
   * @brief Adds a contact to the solver
   * @param contact the contact to add
   */
  PointContact& add_point_contact(PositionTask& position_task);
  PointContact& add_unilateral_point_contact(PositionTask& position_task);
  RelativePointContact& add_relative_point_contact(RelativePositionTask& position_task);
  PlanarContact& add_fixed_contact(FrameTask& frame_task);
  PlanarContact& add_planar_contact(FrameTask& frame_task);

  /**
   * @brief Sets a DoF as passive
   * @param joint_name the DoF name
   * @param is_passive true if the DoF should be passive
   */
  void set_passive(const std::string& joint_name, bool is_passive = true);

  /**
   * @brief Adds a loop closing constraint (xy should be zero)
   * @param frame_a
   * @param frame_b
   */
  void add_loop_closing_constraint(const std::string& frame_a, const std::string& frame_b, const std::string& axises);

  PositionTask& add_position_task(pinocchio::FrameIndex frame_index, Eigen::Vector3d target_world);
  PositionTask& add_position_task(std::string frame_name, Eigen::Vector3d target_world);
  OrientationTask& add_orientation_task(pinocchio::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);
  OrientationTask& add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame);
  FrameTask add_frame_task(pinocchio::FrameIndex frame_index, Eigen::Affine3d T_world_frame);
  FrameTask add_frame_task(std::string frame_name, Eigen::Affine3d T_world_frame);
  RelativePositionTask& add_relative_position_task(pinocchio::FrameIndex frame_a_index,
                                                   pinocchio::FrameIndex frame_b_index, Eigen::Vector3d target_world);
  RelativePositionTask& add_relative_position_task(std::string frame_a_name, std::string frame_b_name,
                                                   Eigen::Vector3d target_world);
  CoMTask& add_com_task(Eigen::Vector3d target_world);
  StaticTask& add_static_task();
  JointsTask& add_joints_task();

  Result solve();

  RobotWrapper& robot;

  void remove_task(Task* task);

  int N;

protected:
  Problem problem;
  std::vector<LoopClosure> loop_closing_constraints;

  std::set<Task*> tasks;

  // Task id (this is only useful when task names are not specified, each task will have an unique ID)
  int task_id = 0;

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
}  // namespace dynamics
}  // namespace placo