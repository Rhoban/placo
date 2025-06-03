#pragma once

#include "placo/model/robot_wrapper.h"
#ifdef HAVE_RHOBAN_UTILS
#include "rhoban_utils/history/history.h"
#endif

namespace placo::humanoid
{
class HumanoidRobot : public model::RobotWrapper
{
public:
  /**
   * @brief Which side the foot is
   */
  enum Side
  {
    Left = 0,
    Right = 1
  };

  static Side string_to_side(const std::string& str);
  static Side other_side(Side side);

  HumanoidRobot(std::string model_directory = "robot", int flags = 0, std::string urdf_content = "");

  void initialize();
  void init_config();

  /**
   * @brief Updates which frame should be the current support
   */
  void update_support_side(Side side);
  void update_support_side(const std::string& side);

  /**
   * @brief Place the robot on its support on the floor
   */
  void ensure_on_floor();

  /**
   * @brief Place the robot on its support on the floor according
   * to the trunk orientation and the kinematic configuration
   * @param R_world_trunk Orientation of the trunk
   */
  void ensure_on_floor_oriented(Eigen::Matrix3d R_world_trunk);

  /**
   * @brief Rotate the robot around its support
   * @param R_world_trunk Orientation of the trunk from the IMU
   */
  void update_from_imu(Eigen::Matrix3d R_world_trunk);

  Eigen::Affine3d get_T_world_left();
  Eigen::Affine3d get_T_world_right();
  Eigen::Affine3d get_T_world_trunk();

  /**
   * @brief Compute the center of mass velocity from the speed of the motors and the orientation of the trunk
   * @param support Support side
   * @param omega_b Trunk angular velocity in the body frame
   * @return Center of mass velocity
   */
  Eigen::Vector3d get_com_velocity(Side support, Eigen::Vector3d omega_b);

  /**
   * @brief Compute the torques of the motors from the contact forces
   * @param acc_a Accelerations of the actuated DoFs
   * @param contact_forces Contact forces from the feet (forces are supposed normal to the ground)
   * @param use_non_linear_effects If true, non linear effects are taken into account (state.qd necessary)
   * @return Torques of the motors
   */
  Eigen::VectorXd get_torques(Eigen::VectorXd acc_a, Eigen::VectorXd contact_forces,
                              bool use_non_linear_effects = false);

  /**
   * @brief Compute the Divergent Component of Motion (DCM)
   * @param omega Natural frequency of the LIP (= sqrt(g/h))
   * @param com_velocity CoM velocity
   * @return DCM
   */
  Eigen::Vector2d dcm(double omega, Eigen::Vector2d com_velocity);

  /**
   * @brief Compute the Zero-tilting Moment Point (ZMP)
   * @param omega Natural frequency of the LIP (= sqrt(g/h))
   * @param com_acceleration CoM acceleration
   * @return ZMP
   */
  Eigen::Vector2d zmp(double omega, Eigen::Vector2d com_acceleration);

  // We suppose we have one support frame and associated transformation
  RobotWrapper::FrameIndex support_frame();
  RobotWrapper::FrameIndex flying_frame();

  /**
   * @brief Get the pan and tilt target for the camera to look at a target position
   * @param pan Pan adress
   * @param tilt Tilt adress
   * @param P_world_target Position in the world referential of the target we want to look at
   * @return True if a solution is found, false if not
   */
  bool camera_look_at(double& pan, double& tilt, const Eigen::Vector3d& P_world_target);

#ifdef HAVE_RHOBAN_UTILS
  /**
   * @brief Load the robot state from an history log
   * @param histories Log file
   * @param timestamp Timestamp
   * @param use_imu Use IMU values for the trunk orientation
   * @param qd_joints If not empty, use these values for the joint velocities (state.qd) (NOT TESTED)
   */
  void read_from_histories(rhoban_utils::HistoryCollection& histories, double timestamp, std::string source = "read",
                           bool use_imu = false, Eigen::VectorXd qd_joints = Eigen::VectorXd::Zero(1));
#endif

  /**
   * @brief Are both feet supporting the robot
   */
  bool support_is_both;

  /**
   * @brief The current side (left or right) associated with T_world_support
   */
  Side support_side;

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
}  // namespace placo::humanoid