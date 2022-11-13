#pragma once

#include "placo/model/mobile_robot.h"
#include <Eigen/Dense>

namespace placo
{
class KinematicsSolver
{
public:
  KinematicsSolver(MobileRobot& robot);

  enum Priority
  {
    Hard = 0,
    Soft = 1
  };

  struct Task
  {
    Task();

    KinematicsSolver* solver;
    std::string name;

    void set_priority_value(Priority priority);
    void set_priority(std::string priority);
    void set_weight(double weight);
    void set_name(std::string name);

    void configure(std::string name, std::string priority = "soft", double weight = 1.0);

    // Task priority (hard: equality constraint, soft: objective function)
    Priority priority;

    // If the task is "soft", this is its weight
    double weight;

    // The task is of type Ax = b
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;

    virtual void update() = 0;
    virtual std::string type_name() = 0;
    virtual std::string error_unit() = 0;
    virtual double error();
  };

  struct PositionTask : public Task
  {
    PositionTask(MobileRobot::FrameIndex frame_index, Eigen::Vector3d target_world);

    MobileRobot::FrameIndex frame_index;
    Eigen::Vector3d target_world;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct RelativePositionTask : public Task
  {
    RelativePositionTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Vector3d target);

    MobileRobot::FrameIndex frame_a;
    MobileRobot::FrameIndex frame_b;
    Eigen::Vector3d target;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct CoMTask : public Task
  {
    CoMTask(Eigen::Vector3d target_world);

    Eigen::Vector3d target_world;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct OrientationTask : public Task
  {
    OrientationTask(MobileRobot::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);

    MobileRobot::FrameIndex frame_index;
    Eigen::Matrix3d R_world_frame;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct RelativeOrientationTask : public Task
  {
    RelativeOrientationTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Matrix3d R_a_b);

    MobileRobot::FrameIndex frame_a;
    MobileRobot::FrameIndex frame_b;
    Eigen::Matrix3d R_a_b;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct FrameTask
  {
    FrameTask(PositionTask& position, OrientationTask& orientation);

    void configure(std::string name, std::string priority = "soft", double position_weight = 1.0,
                   double orientation_weight = 1.0);

    PositionTask& position;
    OrientationTask& orientation;

    Eigen::Affine3d get_T_world_frame() const;
    void set_T_world_frame(Eigen::Affine3d T_world_frame);
  };

  struct RelativeFrameTask
  {
    RelativeFrameTask(RelativePositionTask& position, RelativeOrientationTask& orientation);

    void configure(std::string name, std::string priority = "soft", double position_weight = 1.0,
                   double orientation_weight = 1.0);

    RelativePositionTask& position;
    RelativeOrientationTask& orientation;

    Eigen::Affine3d get_T_a_b() const;
    void set_T_a_b(Eigen::Affine3d T_world_frame);
  };

  struct AxisAlignTask : public Task
  {
    AxisAlignTask(MobileRobot::FrameIndex frame_index, Eigen::Vector3d axis_frame, Eigen::Vector3d targetAxis_world);

    MobileRobot::FrameIndex frame_index;
    Eigen::Vector3d axis_frame;
    Eigen::Vector3d targetAxis_world;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct PoseTask : public Task
  {
    PoseTask(MobileRobot::FrameIndex frame_index, Eigen::Affine3d T_world_frame);

    MobileRobot::FrameIndex frame_index;
    Eigen::Affine3d T_world_frame;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct RelativePoseTask : public Task
  {
    RelativePoseTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, Eigen::Affine3d T_a_b);

    MobileRobot::FrameIndex frame_a;
    MobileRobot::FrameIndex frame_b;
    Eigen::Affine3d T_a_b;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct JointTask : public Task
  {
    JointTask(std::string joint, double target);

    std::string joint;
    double target;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct JointsTask : public Task
  {
    JointsTask();

    std::map<std::string, double> joints;

    void set_joint(std::string joint, double target);

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct DistanceTask : public Task
  {
    DistanceTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, double distance);

    MobileRobot::FrameIndex frame_a;
    MobileRobot::FrameIndex frame_b;
    double distance;

    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  struct RegularizationTask : public Task
  {
    virtual void update();
    virtual std::string type_name();
    virtual std::string error_unit();
  };

  /**
   * @brief Adds a position task
   * @param frame the robot frame we want to control
   * @param target_world the target position, expressed in the world (as T_world_frame)
   */
  PositionTask& add_position_task(MobileRobot::FrameIndex frame, Eigen::Vector3d target_world);
  PositionTask& add_position_task(std::string frame, Eigen::Vector3d target_world);

  /**
   * @brief Adds a relative position task
   * @param frame_a frame a
   * @param frame_b frame b
   * @param target the target vector between frame a and b (expressed in world)
   */
  RelativePositionTask& add_relative_position_task(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b,
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
  OrientationTask& add_orientation_task(MobileRobot::FrameIndex frame, Eigen::Matrix3d R_world_frame);
  OrientationTask& add_orientation_task(std::string frame, Eigen::Matrix3d R_world_frame);

  /**
   * @brief Adds a relative orientation task
   * @param frame_a frame a
   * @param frame_b frame b
   * @param R_a_b the desired orientation
   */
  RelativeOrientationTask& add_relative_orientation_task(MobileRobot::FrameIndex frame_a,
                                                         MobileRobot::FrameIndex frame_b, Eigen::Matrix3d R_a_b);
  RelativeOrientationTask& add_relative_orientation_task(std::string frame_a, std::string frame_b,
                                                         Eigen::Matrix3d R_a_b);

  /**
   * @brief Adds an axis alignment task. The goal here is to keep the given axis (expressed in the given frame) aligned
   * with another one (given in the world)
   * @param frame the robot frame we want to control
   * @param axis_frame the axis to align, expressed in the robot frame
   * @param targetAxis_world the target axis (in the world) we want to be aligned with
   */
  AxisAlignTask& add_axisalign_task(MobileRobot::FrameIndex frame, Eigen::Vector3d axis_frame, Eigen::Vector3d);
  AxisAlignTask& add_axisalign_task(std::string frame, Eigen::Vector3d axis_frame, Eigen::Vector3d target_axis_world);

  /**
   * @brief Adds a frame task, this is equivalent to a position + orientation task, resulting in a "decoupled" style
   * control of a given frame
   * @param frame the robot frame we want to control
   * @param T_world_frame the target for the frame we want to control, expressed in the world (as T_world_frame)
   * @param priority task priority (hard: equality constraint, soft: objective function)
   */
  FrameTask add_frame_task(MobileRobot::FrameIndex frame, Eigen::Affine3d T_world_frame);
  FrameTask add_frame_task(std::string frame, Eigen::Affine3d T_world_frame);

  /**
   * @brief Adds a relative frame task
   * @param frame_a frame a
   * @param frame_b frame b
   * @param T_a_b desired transformation
   */
  RelativeFrameTask add_relative_frame_task(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b,
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
  PoseTask& add_pose_task(MobileRobot::FrameIndex frame, Eigen::Affine3d T_world_frame);
  PoseTask& add_pose_task(std::string frame, Eigen::Affine3d T_world_frame);

  /**
   * @brief Adds a relative pose task, this is similar to add_pose_task, but it ensures a relative pose between two
   * given frames.
   * @param frame_a frame A
   * @param frame_b frame B
   * @param T_a_b relative desired pose to enforce
   */
  RelativePoseTask& add_relative_pose_task(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b,
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
  DistanceTask& add_distance_task(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, double distance);
  DistanceTask& add_distance_task(std::string frame_a, std::string frame_b, double distance);

  /**
   * @brief Adds a regularization task for a given magnitude
   * @param magnitude regularization magnitude
   */
  RegularizationTask& add_regularization_task(double magnitude = 1e-6);

  /**
   * @brief Constructs the QP problem and solves it
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
   * @brief Clears the internal tasks
   */
  void clear_tasks();

  /**
   * @brief Shows the tasks status
   */
  void dump_status();

protected:
  MobileRobot& robot;
  std::set<int> masked_dof;
  std::vector<Task*> tasks;

  template <typename T>
  T& add_task(T* task)
  {
    task->solver = this;
    std::ostringstream oss;
    oss << "Task_" << tasks.size();
    task->name = oss.str();
    tasks.push_back(task);

    return *task;
  }

  /**
   * @brief Size of the problem (number of variables we will search)
   */
  int N;
};
}  // namespace placo