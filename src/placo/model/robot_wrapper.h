#pragma once

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/container/boost-container-limits.hpp>

namespace placo::model
{
/**
 * @brief This class contains the robot model, state, and all convenient methods. All the rigid body algorithms
 * (namely, based on pinocchio) are wrapped in this object
 */
class RobotWrapper
{
public:
  /**
   * @brief Flags passed to the constructor
   */
  enum Flags
  {
    /**
     * @brief The collisions from the URDF will be loaded as visual. This can significantly speed up loading time
     * when we don't want the visual meshes to be loaded.
     */
    COLLISION_AS_VISUAL = 1,

    /**
     * @brief All self-collisions will be ignored (the pairs will be removed)
     */
    IGNORE_COLLISIONS = 2
  };

  /**
   * @brief Creates a robot wrapper from a URDF file.
   * @param model_directory robot model (URDF). It can be a path to an URDF file, or a directory containing an URDF
   * file named 'robot.urdf'
   * @param flags see \ref Flags
   * @param urdf_content if it is not empty, it will be used as the URDF content instead of loading it from the
   * file
   */
  RobotWrapper(std::string model_directory, int flags = 0, std::string urdf_content = "");

  /**
   * @brief Represents the robot state
   *
   * **WARNING:** we always use a floating base as a first DoF, meaning that the size of ``q`` is not the same
   * as the size of ``qd`` and ``qdd``. Use \ref get_joint_offset (for ``q``) and \ref get_joint_v_offset
   * (for ``qd`` and ``qdd``) to get the offset of a given DoF in the state.
   */
  struct State
  {
    /**
     * @brief joints configuration \f$q\f$
     */
    Eigen::VectorXd q;
    /**
     * @brief joints velocity \f$\dot q\f$
     */
    Eigen::VectorXd qd;
    /**
     * @brief joints acceleration \f$\ddot q\f$
     */
    Eigen::VectorXd qdd;
  };

  /**
   * @brief Robot's current state
   */
  State state;

  /**
   * @brief The index of a frame (currently directly wrapped to pinocchio's FrameIndex)
   */
  typedef pinocchio::FrameIndex FrameIndex;

  /**
   * @brief Loads collision pairs from a given JSON file.
   *
   * Here is an example of ``collisions.json`` file format:
   *
   * ```json
   * [
   *   ["body1", "body2"],
   *   ["body1", "body3"],
   *   ["body2", "body3"]
   *   ...
   * ]
   * ```
   *
   * It specifies which bodies are allowed to enter in collisions.
   * Alternatively, pairs can also contain integers representing the index of the bodies in the collision model.
   *
   * @param filename path to collisions.json file
   */
  void load_collision_pairs(const std::string& filename);

  /**
   * @brief Reset internal states, this sets q to the neutral position, qd and qdd to zero
   *
   * Se \ref state
   */
  void reset();

  /**
   * @brief Retrieves a frame index from its name. This is useful to speed-up later calls to methods requiring
   * frames (e.g \ref get_T_world_frame or \ref frame_jacobian)
   * @param frame frame name
   * @return frame index
   */
  FrameIndex get_frame_index(const std::string& frame);

  /**
   * @brief Sets the value of a joint in state.q
   * @param name joint name
   * @param value joint value (e.g rad for revolute or meters for prismatic)
   */
  void set_joint(const std::string& name, double value);

  /**
   * @brief Retrieves a joint value from state.q
   * @param name joint name
   * @return the joint current (inner state) value (e.g rad for revolute or meters for prismatic)
   */
  double get_joint(const std::string& name);

  /**
   * @brief Sets the joint velocity in state.qd
   * @param name joint name
   * @param value joint velocity
   */
  void set_joint_velocity(const std::string& name, double value);

  /**
   * @brief Gets the joint velocity from state.qd
   * @param name joint name
   * @return joint velocity
   */
  double get_joint_velocity(const std::string& name);

  /**
   * @brief Sets the joint acceleration in state.qd
   * @param name joint name
   * @param value joint acceleration
   */
  void set_joint_acceleration(const std::string& name, double value);

  /**
   * @brief Gets the joint acceleration from state.qd
   * @param name joint name
   * @return joint acceleration
   */
  double get_joint_acceleration(const std::string& name);

  /**
   * @brief Gets the offset for a given joint in the \ref state (in \ref State.q)
   * @param name joint name
   * @return offset in state.q
   */
  int get_joint_offset(const std::string& name);

  /**
   * @brief Gets the offset for a given joint in the \ref state (in \ref State.qd and \ref State.qdd)
   * @param name joint name
   * @return offset in state.qd and state.qdd
   */
  int get_joint_v_offset(const std::string& name);

  /**
   * @brief Sets the limits for a given joint.
   *
   * By default, the joint limits are loaded from the URDF file, this method can be used to override them.
   *
   * @param name joint name
   * @param lower lower limit
   * @param upper upper limit
   */
  void set_joint_limits(const std::string& name, double lower, double upper);

  /**
   * @brief Sets the velocity limit for a given joint.
   *
   * By default, the joint limits are loaded from the URDF file, this method can be used to override them.
   *
   * @param name joint name
   * @param limit joint limit
   */
  void set_velocity_limit(const std::string& name, double limit);

  /**
   * @brief Gets the limits for a given joint.
   *
   * @param name joint name
   * @return pair of (lower, upper) limits
   */
  std::pair<double, double> get_joint_limits(const std::string& name);

  /**
   * @brief Set the velocity limits for **all** the joints
   * @param limit limit
   */
  void set_velocity_limits(double limit);

  /**
   * @brief Sets the torque limit for a given joint.
   *
   * By default, the joint limits are loaded from the URDF file, this method can be used to override them.
   *
   * @param name joint name
   * @param limit torque limit
   */
  void set_torque_limit(const std::string& name, double limit);

  /**
   * @brief builds a neutral state (neutral position, zero speed)
   * @return the state
   */
  State neutral_state();

  /**
   * @brief Update internal computation for kinematics (frames, jacobian). This method should be called when
   * the robot state has changed.
   */
  void update_kinematics();

  /**
   * @brief Compute kinematics hessians
   */
  void compute_hessians();

  /**
   * @brief Get the component for the hessian of a given frame for a given joint
   */
  Eigen::MatrixXd get_frame_hessian(FrameIndex frame, int joint_v_index);

  /**
   * @brief Gets the CoM position in the world
   */
  Eigen::Vector3d com_world();

  /**
   * @brief Gets the frame to world transformation matrix for a given frame
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param index frame index
   * @return transformation
   */
  Eigen::Affine3d get_T_world_frame(FrameIndex index);

  /**
   * @brief Gets the frame to world transformation matrix for a given frame
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame frame
   * @return transformation
   */
  Eigen::Affine3d get_T_world_frame(const std::string& frame);

  /**
   * @brief Gets the transformation matrix from frame b to a
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param index_a frame a
   * @param index_b frame b
   * @return transformation
   */
  Eigen::Affine3d get_T_a_b(FrameIndex index_a, FrameIndex index_b);

  /**
   * @brief Gets the transformation matrix from frame b to a
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame_a frame a
   * @param frame_b frame b
   * @return transformation
   */
  Eigen::Affine3d get_T_a_b(const std::string& frame_a, const std::string& frame_b);

  /**
   * @brief Returns the transformation matrix from the fbase frame (which is the root of the URDF) to
   * the world
   * @return transformation
   */
  Eigen::Affine3d get_T_world_fbase();

  /**
   * @brief Updates the floating base to match the given transformation matrix
   * @param T_world_fbase transformation matrix
   */
  void set_T_world_fbase(Eigen::Affine3d T_world_fbase);

  /**
   * @brief Updates the floating base status so that the given frame has the given transformation matrix.
   *
   * This method is convenient to place the robot in a given position in the world.
   *
   * @param frame frame to update
   * @param T_world_frameTarget transformation matrix
   */
  void set_T_world_frame(FrameIndex frame, Eigen::Affine3d T_world_frameTarget);

  /**
   * @brief Updates the floating base status so that the given frame has the given transformation matrix.
   *
   * This method is convenient to place the robot in a given position in the world.
   *
   * @param frame frame to update
   * @param T_world_frameTarget transformation matrix
   */
  void set_T_world_frame(const std::string& frame, Eigen::Affine3d T_world_frameTarget);

  /**
   * @brief Represents a collision between two bodies
   */
  struct Collision
  {
    /**
     * @brief Index of object A in the collision geometry
     */
    int objA;

    /**
     * @brief Index of object B in the collision geometry
     */
    int objB;

    /**
     * @brief The joint parent of body A
     */
    pinocchio::JointIndex parentA;

    /**
     * @brief The joint parent of body B
     */
    pinocchio::JointIndex parentB;

    /**
     * @brief Name of the body A
     */
    std::string bodyA;

    /**
     * @brief Name of the body B
     */
    std::string bodyB;

    /**
     * @brief Contact points
     */
    std::vector<Eigen::Vector3d> contacts;

    bool operator==(const Collision& other);
  };

  /**
   * @brief Finds the self collision in current state, if ``stop_at_first`` is true, it will stop at the first
   * collision found
   * @param stop_at_first whether to stop at the first collision found
   * @return a vector of \ref Collision
   */
  std::vector<Collision> self_collisions(bool stop_at_first = false);

  /**
   * @brief Represents a distance between two bodies
   */
  struct Distance
  {
    /**
     * @brief Index of object A in the collision geometry
     */
    int objA;

    /**
     * @brief Index of object B in the collision geometry
     */
    int objB;

    /**
     * @brief Parent joint of body A
     */
    pinocchio::JointIndex parentA;

    /**
     * @brief Parent joint of body B
     */
    pinocchio::JointIndex parentB;

    /**
     * @brief Point of object A considered
     */
    Eigen::Vector3d pointA;

    /**
     * @brief Point of object B considered
     */
    Eigen::Vector3d pointB;

    /**
     * @brief Current minimum distance between the two objects
     */
    double min_distance;

    bool operator==(const Distance& other);
  };

  /**
   * @brief Computes all minimum distances between current collision pairs
   *
   * This can be used for collision avoidance (e.g in \ref kinematics::KinematicsSolver and \ref
   * dynamics::DynamicsSolver)
   *
   * @return vector of \ref Distance
   */
  std::vector<Distance> distances();

  /**
   * @brief Frame jacobian, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame the frame for which we want the jacobian
   * @return jacobian (6 x nv matrix), where nv is the size of ``qd``
   */
  Eigen::MatrixXd frame_jacobian(FrameIndex frame,
                                 pinocchio::ReferenceFrame ref = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Frame jacobian, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame the frame for which we want the jacobian
   * @return jacobian (6 x nv matrix), where nv is the size of ``qd``
   */
  Eigen::MatrixXd frame_jacobian(const std::string& frame, const std::string& reference = "local_world_aligned");

  /**
   * @brief Jacobian time variation \f$\dot J\f$, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame the frame for which we want the jacobian time variation
   * @param reference the reference frame
   * @return jacobian time variation (6 x nv matrix), where nv is the size of ``qd``
   */
  Eigen::MatrixXd frame_jacobian_time_variation(
      FrameIndex frame, pinocchio::ReferenceFrame ref = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Jacobian time variation \f$\dot J\f$, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame the frame for which we want the jacobian time variation
   * @param reference the reference frame
   * @return jacobian time variation (6 x nv matrix), where nv is the size of ``qd``
   */
  Eigen::MatrixXd frame_jacobian_time_variation(const std::string& frame, const std::string& reference = "local_world_"
                                                                                                         "aligned");

  /**
   * @brief Joint jacobian, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame the frame for which we want the jacobian
   * @return jacobian (6xn matrix)
   * @pyignore
   */
  Eigen::MatrixXd joint_jacobian(pinocchio::JointIndex joint,
                                 pinocchio::ReferenceFrame ref = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Joint jacobian, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame the frame for which we want the jacobian
   * @return jacobian (6xn matrix)
   */
  Eigen::MatrixXd joint_jacobian(const std::string& joint, const std::string& reference = "local_world_aligned");

  /**
   * @brief Joint jacobian time variation \f$\dot J\f$, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param joint the joint for which we want the jacobian time variation
   * @param ref the reference frame
   * @return jacobian time variation (6xn matrix)
   */
  Eigen::MatrixXd joint_jacobian_time_variation(
      pinocchio::JointIndex joint, pinocchio::ReferenceFrame ref = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

  /**
   * @brief Joint jacobian time variation \f$\dot J\f$, default reference is LOCAL_WORLD_ALIGNED
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   *
   * @param joint the joint for which we want the jacobian time variation
   * @param ref the reference frame
   * @return jacobian time variation (6xn matrix)
   */
  Eigen::MatrixXd joint_jacobian_time_variation(const std::string& joint, const std::string& reference = "local_world_"
                                                                                                         "aligned");

  /**
   * @brief Jacobian of the relative position of the position of b expressed in a
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame_a frame index A
   * @param frame_b frame index B
   * @return relative position jacobian of b expressed in a (3 x n matrix)
   */
  Eigen::MatrixXd relative_position_jacobian(FrameIndex frame_a, FrameIndex frame_b);

  /**
   * @brief Jacobian of the relative position of the position of b expressed in a
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @param frame_a frame A
   * @param frame_b frame B
   * @return relative position jacobian of b expressed in a (3 x n matrix)
   */
  Eigen::MatrixXd relative_position_jacobian(const std::string& frame_a, const std::string& frame_b);

  /**
   * @brief Jacobian of the CoM position expressed in the world
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @return jacobian (3 x n matrix)
   */
  Eigen::Matrix3Xd com_jacobian();

  /**
   * @brief Jacobian time variation of the CoM expressed in the world
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @return jacobian (3 x n matrix)
   */
  Eigen::Matrix3Xd com_jacobian_time_variation();

  /**
   * @brief Centroidal map
   *
   * Be sure you called \ref update_kinematics before calling this method if your state has changed
   *
   * @return jacobian (6 x n matrix)
   */
  Eigen::MatrixXd centroidal_map();

  /**
   * @brief Computes generalized gravity
   */
  Eigen::VectorXd generalized_gravity();

  /**
   * @brief Computes non-linear effects (Corriolis, centrifual and gravitationnal effects)
   */
  Eigen::VectorXd non_linear_effects();

  /**
   * @brief Updates the rotor inertia (used for apparent inertia computation in the dynamics)
   */
  void set_rotor_inertia(const std::string& joint_name, double rotor_inertia);

  /**
   * @brief Updates the rotor gear ratio (used for apparent inertia computation in the dynamics)
   */
  void set_gear_ratio(const std::string& joint_name, double rotor_gear_ratio);

  /**
   * @brief Computes the mass matrix
   */
  Eigen::MatrixXd mass_matrix();

  /**
   * @brief Sets the gravity vector
   */
  void set_gravity(Eigen::Vector3d gravity);

  /**
   * @brief Integrates the internal \ref state for a given ``dt``
   *
   * This will first update qd from qdd, and then q from qd
   *
   * @param dt delta time for integration expressed in seconds
   */
  void integrate(double dt);

  /**
   * @brief Computes torques needed by the robot to compensate for the generalized gravity, assuming that the given
   * frame is the (only) contact supporting the robot
   *
   * @param frame frame index
   */
  Eigen::VectorXd static_gravity_compensation_torques(FrameIndex frame);

  /**
   * @brief Computes torques needed by the robot to compensate for the generalized gravity, assuming that the given
   * frame is the (only) contact supporting the robot
   *
   * @param frame frame
   */
  Eigen::VectorXd static_gravity_compensation_torques(std::string frame);

  /**
   * @brief Computes required torques in the robot DOFs for a given acceleration of the actuated DOFs, assuming
   * that the given frame is fixed
   *
   * @param qdd_a acceleration of the actuated DOFs
   * @param fixed_frame frame index
   */
  Eigen::VectorXd torques_from_acceleration_with_fixed_frame(Eigen::VectorXd qdd_a, FrameIndex fixed_frame);

  /**
   * @brief Computes required torques in the robot DOFs for a given acceleration of the actuated DOFs, assuming
   * that the given frame is fixed
   *
   * @param qdd_a acceleration of the actuated DOFs
   * @param fixed_frame frame
   */
  Eigen::VectorXd torques_from_acceleration_with_fixed_frame(Eigen::VectorXd qdd_a, std::string fixed_frame);

  /**
   * @brief All the joint names
   *
   * @param include_floating_base whether to include the floating base joint (false by default)
   */
  std::vector<std::string> joint_names(bool include_floating_base = false);

  /**
   * @brief All the frame names
   */
  std::vector<std::string> frame_names();

  /**
   * @brief Total mass
   */
  double total_mass();

  /**
   * @brief Adds some noise to the configuration
   */
  void add_q_noise(double noise);

  /**
   * @brief URDF model directory
   */
  std::string model_directory;

  /**
   * @brief Pinocchio model
   */
  pinocchio::Model model;

  /**
   * @brief Pinocchio collision model
   */
  pinocchio::GeometryModel collision_model;

  /**
   * @brief Pinocchio visual model
   */
  pinocchio::GeometryModel visual_model;

  /**
   * @brief Pinocchio model data
   */
  pinocchio::Data* data;

protected:
  /**
   * @brief Free flyer joint
   */
  pinocchio::JointModelFreeFlyer root_joint;
};
}  // namespace placo::model