#pragma once

#include "placo/model/robot_wrapper.h"
#include "rhoban_utils/history/history.h"

namespace placo
{
class HumanoidRobot : public RobotWrapper
{
public:
  /**
   * @brief Which side the foot is
   */
  enum Side
  {
    Left = 0,
    Right = 1,
    Both = 2
  };

  static Side string_to_side(const std::string& str);
  static Side other_side(Side side);

  HumanoidRobot(std::string model_directory = "robot");

  void initialize();
  void init_config();

  /**
   * @brief Updates which frame should be the current support
   */
  void update_support_side(Side side);
  void update_support_side(const std::string& side);
  void ensure_on_floor();

  Eigen::Affine3d get_T_world_left();
  Eigen::Affine3d get_T_world_right();
  Eigen::Affine3d get_T_world_trunk();

  // Gets the self frame in world, ie the weighted average of the 2 foot frames
  Eigen::Affine3d get_T_world_self();

  /// @brief Compute the center of mass velocity from the speed of the motors and the orientation of the trunk
  /// @param qd_a Velocity of the actuated dofs
  /// @param support Support side
  /// @param d_roll Trunk roll speed
  /// @param d_pitch Trunk pitch speed
  /// @param d_yaw Trunk yaw speed
  /// @return Center of mass velocity
  Eigen::Vector3d get_com_velocity(Eigen::VectorXd qd_a, Side support, double d_roll, double d_pitch, double d_yaw);

  // We suppose we have one support frame and associated transformation
  RobotWrapper::FrameIndex support_frame();
  RobotWrapper::FrameIndex flying_frame();

  /// @brief Get the pan and tilt target for the camera to look at a targeted position
  /// @param pan Pan adress
  /// @param tilt Tilt adress
  /// @param P_world_target Position in the world referential of the target we want to look at
  /// @return True if a solution is found, false if not
  bool camera_look_at(double& pan, double& tilt, const Eigen::Vector3d& P_world_target);

  void update_trunk_orientation(double roll, double pitch, double yaw);

  /// @brief Load the robot state from an history log
  /// @param histories Log file
  /// @param timestamp Timestamp
  /// @param use_imu Use IMU values for the trunk orientation
  void readFromHistories(rhoban_utils::HistoryCollection& histories, double timestamp, bool use_imu = false);

  /**
   * @brief The current side (left, right or both) supporting the robot
   */
  Side support_side;

  /**
   * @brief The current flying foot or the next flying foot if the support_side is both
   */
  Side flying_side;

  /**
   * @brief Transformation from support to world
   */
  Eigen::Affine3d T_world_support;

  // Useful frames
  RobotWrapper::FrameIndex left_foot;
  RobotWrapper::FrameIndex right_foot;
  RobotWrapper::FrameIndex trunk;

  // Useful distances based on the URDF
  double dist_z_pan_tilt;    // Distance along z between the pan DoF and the tilt DoF in the head
  double dist_z_pan_camera;  // Distance along z between the pan DoF and the camera in the head
  double dist_y_trunk_foot;  // Distance along y between the trunk and the left_foot frame in model
};
}  // namespace placo