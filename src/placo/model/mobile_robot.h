#pragma once

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/container/boost-container-limits.hpp>

namespace placo {
class MobileRobot {
public:
  MobileRobot(std::string model_directory = "robot/");

  /**
   * @brief Load collision pairs from the given json file
   * @param filename
   */
  void load_collisions_pairs(const std::string &filename);

  /**
   * @brief Reset internal states
   */
  void reset();

  /**
   * @brief Retrieves the frame index
   * @param frame
   * @return a frame index
   */
  pinocchio::FrameIndex get_frame_index(const std::string &frame);

  /**
   * @brief Set joint values
   * @param name DOF name
   * @param value DOF value
   */
  void set_joint(const std::string &name, double value);

  /**
   * @brief Retrieves a joint valud
   * @param name
   * @return the joint current (inner state) value
   */
  double get_joint(const std::string &name);

  /**
   * @brief Gets the offset for a given joint in the state
   * @param name joint name
   * @return offset in state.q
   */
  int get_joint_offset(const std::string &name);

  /**
   * @brief Gets the offset for a given joint speed in the state
   * @param name joint name
   * @return offset in state.qd
   */
  int get_joint_v_offset(const std::string &name);

  /**
   * @brief List of expected DOFs to be present, throws an error when loading
   * URDF if some is missing
   * @return the vector of (string) DOF names
   */
  virtual std::vector<std::string> expected_dofs();

  /**
   * @brief List of expected frames to be present, throws an error when loading
   * URDF if some is missing
   * @return the vector of (string ) frame names
   */
  virtual std::vector<std::string> expected_frames();

  // Robot state
  struct State {
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
  };

  /**
   * @brief Robot's current state
   */
  State state;

  /**
   * @brief builds a neutral state (neutral position, zero speed)
   * @return the state
   */
  State neutral_state();

  /**
   * @brief Update the current kinematics
   */
  void update_kinematics();

  /**
   * @brief Returns the transformation matrix of the floating base in the world
   * (from internal q state)
   * @return floating base to the world frame
   */
  Eigen::Affine3d get_T_world_fbase();

  /**
   * @brief Updates the floating base to match the given transformation matrix
   * @param T
   */
  void set_T_world_fbase(Eigen::Affine3d T);

  /**
   * @brief Gets the CoM position in the world
   */
  Eigen::Vector3d com_world();

  /**
   * @brief Gets the current transformation (frame to world)
   */
  Eigen::Affine3d get_T_world_frame(const std::string &frame);
  Eigen::Affine3d get_T_world_frame(pinocchio::FrameIndex index);

  /**
   * @brief Updates the floating base so that a given frame matches the provided
   * world transformation.
   */
  void set_T_world_frame(const std::string &frame,
                         Eigen::Affine3d T_world_frameTarget);
  void set_T_world_frame(pinocchio::FrameIndex frame,
                         Eigen::Affine3d T_world_frameTarget);

  struct Collision {
    std::string bodyA;
    std::string bodyB;
    std::vector<Eigen::Vector3d> contacts;

    bool operator==(const Collision &other);
  };

  /**
   * @brief Find self collisions in current state
   * @param stop_at_first if passed to true, stops at first collision found (to
   * reduce computation time)
   * @return a vector of all self collisions
   */
  std::vector<Collision> self_collisions(bool stop_at_first = false);

  /**
   * @brief Computes frame jacobian, default reference is LOCAL WORLD ALIGNED
   * @param frame given frame
   * @return jacobian (6xn matrix)
   */
  Eigen::MatrixXd frame_jacobian(const std::string &frame);
  Eigen::MatrixXd
  frame_jacobian(pinocchio::FrameIndex frame,
                 pinocchio::ReferenceFrame ref =
                     pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Computes the CoM jacobian
   * @return jacobian (3xn matrix)
   */
  Eigen::Matrix3Xd com_jacobian();

  /**
   * @brief Return all the joint names
   */
  std::vector<std::string> joint_names();

  /**
   * @brief Return all the frame names
   */
  std::vector<std::string> frame_names();

  // Pinocchio model
  pinocchio::Model model;
  pinocchio::GeometryModel collision_model;
  pinocchio::GeometryModel visual_model;

protected:
  // Model data
  pinocchio::Data *data;

  // Root free-flyer joint
  pinocchio::JointModelFreeFlyer root_joint;
};
} // namespace placo