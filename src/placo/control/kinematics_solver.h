#pragma once
#include <Eigen/Dense>

#include "placo/model/robot_wrapper.h"

// Tasks
#include "placo/control/task.h"
#include "placo/control/position_task.h"
#include "placo/control/orientation_task.h"
#include "placo/control/frame_task.h"
#include "placo/control/pose_task.h"
#include "placo/control/relative_position_task.h"
#include "placo/control/relative_orientation_task.h"
#include "placo/control/relative_frame_task.h"
#include "placo/control/relative_pose_task.h"
#include "placo/control/com_task.h"
#include "placo/control/axis_align_task.h"
#include "placo/control/axis_plane_task.h"
#include "placo/control/distance_task.h"
#include "placo/control/joint_task.h"
#include "placo/control/joints_task.h"
#include "placo/control/regularization_task.h"
#include "placo/control/centroidal_momentum_task.h"

namespace placo
{
class KinematicsSolver
{
public:
  KinematicsSolver(RobotWrapper& robot_);
  KinematicsSolver(RobotWrapper* robot_);

  /**
   * @brief Adds a position task
   * @param frame the robot frame we want to control
   * @param target_world the target position, expressed in the world (as T_world_frame)
   */
  PositionTask& add_position_task(RobotWrapper::FrameIndex frame, Eigen::Vector3d target_world);
  PositionTask& add_position_task(std::string frame, Eigen::Vector3d target_world);

  /**
   * @brief Adds a relative position task
   * @param frame_a frame a
   * @param frame_b frame b
   * @param target the target vector between frame a and b (expressed in world)
   */
  RelativePositionTask& add_relative_position_task(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b,
                                                   Eigen::Vector3d target);
  RelativePositionTask& add_relative_position_task(std::string frame_a, std::string frame_b, Eigen::Vector3d target);

  /**
   * @brief Adds a com position task
   * @param targetCom_world the target position, expressed in the world (as T_world_frame)
   */
  CoMTask& add_com_task(Eigen::Vector3d targetCom_world);

  /**
   * @brief Adds an orientation task
   * @param frame the robot frame we want to control
   * @param R_world_frame the target orientation we want to achieve, expressed in the world (as T_world_frame)
   */
  OrientationTask& add_orientation_task(RobotWrapper::FrameIndex frame, Eigen::Matrix3d R_world_frame);
  OrientationTask& add_orientation_task(std::string frame, Eigen::Matrix3d R_world_frame);

  /**
   * @brief Adds a relative orientation task
   * @param frame_a frame a
   * @param frame_b frame b
   * @param R_a_b the desired orientation
   */
  RelativeOrientationTask& add_relative_orientation_task(RobotWrapper::FrameIndex frame_a,
                                                         RobotWrapper::FrameIndex frame_b, Eigen::Matrix3d R_a_b);
  RelativeOrientationTask& add_relative_orientation_task(std::string frame_a, std::string frame_b,
                                                         Eigen::Matrix3d R_a_b);

  /**
   * @brief Adds an axis alignment task. The goal here is to keep the given axis (expressed in the given frame) aligned
   * with another one (given in the world)
   * @param frame the robot frame we want to control
   * @param axis_frame the axis to align, expressed in the robot frame
   * @param targetAxis_world the target axis (in the world) we want to be aligned with
   */
  AxisAlignTask& add_axisalign_task(RobotWrapper::FrameIndex frame, Eigen::Vector3d axis_frame,
                                    Eigen::Vector3d targetAxis_world);
  AxisAlignTask& add_axisalign_task(std::string frame, Eigen::Vector3d axis_frame, Eigen::Vector3d targetAxis_world);

  /**
   * @brief Adds an axis plane task (the target axis should lie in the target plane)
   * @param frame the frame
   * @param axis_frame axis expressed in frame
   * @param normal_world normal expressed in world
   */
  AxisPlaneTask& add_axisplane_task(RobotWrapper::FrameIndex frame, Eigen::Vector3d axis_frame,
                                    Eigen::Vector3d normal_world);
  AxisPlaneTask& add_axisplane_task(std::string frame, Eigen::Vector3d axis_frame, Eigen::Vector3d normal_world);

  /**
   * @brief Adds a frame task, this is equivalent to a position + orientation task, resulting in a "decoupled" style
   * control of a given frame
   * @param frame the robot frame we want to control
   * @param T_world_frame the target for the frame we want to control, expressed in the world (as T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective function)
   */
  FrameTask add_frame_task(RobotWrapper::FrameIndex frame, Eigen::Affine3d T_world_frame = Eigen::Affine3d::Identity());
  FrameTask add_frame_task(std::string frame, Eigen::Affine3d T_world_frame = Eigen::Affine3d::Identity());

  /**
   * @brief Adds a relative frame task
   * @param frame_a frame a
   * @param frame_b frame b
   * @param T_a_b desired transformation
   */
  RelativeFrameTask add_relative_frame_task(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b,
                                            Eigen::Affine3d T_a_b);
  RelativeFrameTask add_relative_frame_task(std::string frame_a, std::string frame_b, Eigen::Affine3d T_a_b);

  /**
   * @brief Adds a pose task. The difference with the frame task is that the error will be computed in a coupled way
   * (using the log of the matrix error), resulting in a "screw" target motion. This should be used instead of
   * add_frame_task when the frame is not attached to a particular point we want to control, but to a body. As a result,
   * the position and orientation tasks can't be weighted independently when this task is soft.
   * @param frame the robot frame we want to control
   * @param T_world_frame the target for the frame we want to control, expressed in the world (as T_world_frame)
   */
  PoseTask& add_pose_task(RobotWrapper::FrameIndex frame, Eigen::Affine3d T_world_frame);
  PoseTask& add_pose_task(std::string frame, Eigen::Affine3d T_world_frame);

  /**
   * @brief Adds a relative pose task, this is similar to add_pose_task, but it ensures a relative pose between two
   * given frames.
   * @param frame_a frame A
   * @param frame_b frame B
   * @param T_a_b relative desired pose to enforce
   */
  RelativePoseTask& add_relative_pose_task(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b,
                                           Eigen::Affine3d T_a_b);
  RelativePoseTask& add_relative_pose_task(std::string frame_a, std::string frame_b, Eigen::Affine3d T_a_b);

  /**
   * @brief Adds a joint task, meaning that we want to bring a joint to a given target position
   * @param joint robot joint
   * @param target its target value
   */
  JointTask& add_joint_task(std::string joint, double target);

  /**
   * @brief Adding joints task
   * @param joints value for the joints
   */
  JointsTask& add_joints_task(std::map<std::string, double>& joints);
  JointsTask& add_joints_task();

  /**
   * @brief Adds a distance task to be maintained between two frames
   * @param frame_a frame a
   * @param frame_b frame b
   * @param distance distance to maintain
   */
  DistanceTask& add_distance_task(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b, double distance);
  DistanceTask& add_distance_task(std::string frame_a, std::string frame_b, double distance);

  /**
   * @brief Adding a centroidal momentum task
   * @param L_world
   * @return desired centroidal angular momentum in the world
   */
  CentroidalMomentumTask& add_centroidal_momentum_task(Eigen::Vector3d L_world);

  /**
   * @brief Adds a regularization task for a given magnitude
   * @param magnitude regularization magnitude
   */
  RegularizationTask& add_regularization_task(double magnitude = 1e-6);

  /**
   * @brief Constructs the QP problem and solves it
   * @param apply apply the solution to the robot model
   * @return the vector containing delta q, which are target variations for the robot degrees of freedom.
   */
  Eigen::VectorXd solve(bool apply = false);

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

  /**
   * @brief Clears the internal tasks
   */
  void clear_tasks();

  /**
   * @brief Removes a task from the solver
   */
  void remove_task(Task* task);

  /**
   * @brief Shows the tasks status
   */
  void dump_status();

  /**
   * @brief Configure limits that are enabled
   */
  void configure_limits(bool dofs_limit, bool speeds_limit, bool speed_post_limits);

  /**
   * @brief Enables or disable the self collision inequalities
   * @param enable whether to enable the self collision inequalities
   * @param margin margin that will be used [m]
   * @param trigger the trigger distance at which the inequalities are enabled [m]
   */
  void enable_self_collision_inequalities(bool enable, double margin = 0.005, double trigger = 0.01);

  /**
   * @brief The robot controlled by this solver
   */
  RobotWrapper* robot;

  /**
   * @brief Size of the problem (number of variables)
   */
  int N;

  /**
   * @brief Some configuration noise added before solving
   */
  double noise = 1e-4;

  /**
   * @brief solver dt (for speeds limiting)
   */
  double dt = 0.01;

protected:
  struct Inequality
  {
    // Inequality of the form
    // Ax <= b
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
  };
  std::vector<Inequality> inequalities;

  std::set<int> masked_dof;
  bool masked_fbase;
  std::set<Task*> tasks;

  Eigen::VectorXi activeSet;
  size_t activeSetSize;

  // Modes to limit the DoFs
  bool dofs_limit = true;
  bool speeds_limit = false;
  bool speed_post_limits = false;

  // Self collision prevention
  bool avoid_self_collisions = false;
  double self_collisions_margin = 0.005;  // [m]
  double self_collisions_trigger = 0.01;  // [m]

  void compute_limits_inequalities();
  void compute_self_collision_inequalities();

  template <typename T>
  T& add_task(T* task)
  {
    task->solver = this;
    std::ostringstream oss;
    oss << "Task_" << tasks.size();
    task->name = oss.str();
    tasks.insert(task);

    return *task;
  }
};
}  // namespace placo