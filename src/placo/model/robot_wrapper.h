#pragma once

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/container/boost-container-limits.hpp>

namespace placo
{
class RobotWrapper
{
public:
  enum Flags
  {
    COLLISION_AS_VISUAL=1
  };

  RobotWrapper(std::string model_directory, int flags = 0);

  /**
   * @brief The index of a frame (currently directly wrapped to pinocchio's FrameIndex)
   */
  typedef pinocchio::FrameIndex FrameIndex;

  /**
   * @brief Load collision pairs from the given json file
   * @param filename
   */
  void load_collisions_pairs(const std::string& filename);

  /**
   * @brief Reset internal states
   */
  void reset();

  /**
   * @brief Retrieves the frame index
   * @param frame
   * @return a frame index
   */
  FrameIndex get_frame_index(const std::string& frame);

  /**
   * @brief Set joint values
   * @param name DOF name
   * @param value DOF value [rad]
   */
  void set_joint(const std::string& name, double value);

  /**
   * @brief Retrieves a joint value
   * @param name
   * @return the joint current (inner state) value in radians
   */
  double get_joint(const std::string& name);

  /**
   * @brief Gets the offset for a given joint in the state
   * @param name joint name
   * @return offset in state.q
   */
  int get_joint_offset(const std::string& name);

  /**
   * @brief Gets the offset for a given joint speed in the state
   * @param name joint name
   * @return offset in state.qd
   */
  int get_joint_v_offset(const std::string& name);

  /**
   * @brief Check that expected DOFs and frames are present (see expected_dofs() and expected_frames())
   */
  void check_expected();

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
  struct State
  {
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
  Eigen::Affine3d get_T_world_frame(const std::string& frame);
  Eigen::Affine3d get_T_world_frame(FrameIndex index);

  /**
   * @brief Gets transform from one given frame to another
   */
  Eigen::Affine3d get_T_a_b(const std::string& frame_a, const std::string& frame_b);
  Eigen::Affine3d get_T_a_b(FrameIndex index_a, FrameIndex index_b);

  /**
   * @brief Updates the floating base so that a given frame matches the provided
   * world transformation.
   */
  void set_T_world_frame(const std::string& frame, Eigen::Affine3d T_world_frameTarget);
  void set_T_world_frame(FrameIndex frame, Eigen::Affine3d T_world_frameTarget);

  struct Collision
  {
    int objA, objB;

    // Parent (joints)
    pinocchio::JointIndex parentA;
    pinocchio::JointIndex parentB;
    // Bodies
    std::string bodyA;
    std::string bodyB;
    std::vector<Eigen::Vector3d> contacts;

    bool operator==(const Collision& other);
  };

  /**
   * @brief Find self collisions in current state
   * @param stop_at_first if passed to true, stops at first collision found (to
   * reduce computation time)
   * @return a vector of all self collisions
   */
  std::vector<Collision> self_collisions(bool stop_at_first = false);

  struct Distance
  {
    int objA, objB;

    pinocchio::JointIndex parentA;
    pinocchio::JointIndex parentB;
    Eigen::Vector3d pointA;
    Eigen::Vector3d pointB;
    Eigen::Vector3d normal;
    double min_distance;

    bool operator==(const Distance& other);
  };

  std::vector<Distance> distances();

  /**
   * @brief Computes frame jacobian, default reference is LOCAL_WORLD_ALIGNED
   * @param frame given frame
   * @return jacobian (6xn matrix)
   */
  Eigen::MatrixXd frame_jacobian(const std::string& frame, const std::string& reference = "local_world_aligned");
  Eigen::MatrixXd frame_jacobian(FrameIndex frame,
                                 pinocchio::ReferenceFrame ref = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Computes joint jacobian, default reference is LOCAL_WORLD_ALIGNED
   * @param frame given frame
   * @return jacobian (6xn matrix)
   */
  Eigen::MatrixXd joint_jacobian(const std::string& joint, const std::string& reference = "local_world_aligned");
  Eigen::MatrixXd joint_jacobian(pinocchio::JointIndex joint,
                                 pinocchio::ReferenceFrame ref = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Computes the CoM jacobian
   * @return jacobian (3xn matrix)
   */
  Eigen::Matrix3Xd com_jacobian();

  /**
   * @brief Computes the centroidal map
   * @return jacobian (6xn matrix)
   */
  Eigen::MatrixXd centroidal_map();

  /**
   * @brief Computes generalized gravity
   */
  Eigen::VectorXd generalized_gravity();

  /**
   * @brief Computes non-linear effects
   */
  Eigen::VectorXd non_linear_effects();

  /**
   * @brief Computes the mass matrix
   */
  Eigen::MatrixXd mass_matrix();

  /**
   * @brief Integrate the velocity for a given dt
   */
  void integrate(double dt);

  /**
   * @brief Computes torques needed by the robot to compensate for the generalized gravity, assuming that the given
   * frame is the (only) contact supporting the robot
   *
   * Dimension of the output is nv
   */
  Eigen::VectorXd static_gravity_compensation_torques(FrameIndex frame);
  Eigen::VectorXd static_gravity_compensation_torques(std::string frame);

  /**
   * @brief Return all the joint names
   */
  std::vector<std::string> joint_names();

  /**
   * @brief Return all the actuated joint names
   */
  std::vector<std::string> actuated_joint_names();

  /**
   * @brief Return all the frame names
   */
  std::vector<std::string> frame_names();

  // Pinocchio model
  std::string model_directory;
  pinocchio::Model model;
  pinocchio::GeometryModel collision_model;
  pinocchio::GeometryModel visual_model;

  // Model data
  pinocchio::Data* data;

protected:
  // Root free-flyer joint
  pinocchio::JointModelFreeFlyer root_joint;
};
}  // namespace placo