#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "placo/model/spatial.h"
#include "placo/model/joint.h"

namespace placo::model
{
/**
 * @brief A frame attached to a joint of the kinematic tree
 */
struct Frame
{
  enum Type
  {
    /**
     * @brief Operational frame (user defined)
     */
    OP_FRAME = 1,

    /**
     * @brief Frame of a joint (placed at the joint)
     */
    JOINT = 2,

    /**
     * @brief Frame of a fixed joint
     */
    FIXED_JOINT = 4,

    /**
     * @brief Frame of a body (link)
     */
    BODY = 8,

    /**
     * @brief Any frame type (used for lookups)
     */
    ANY = 15
  };

  /**
   * @brief Frame name
   */
  std::string name;

  /**
   * @brief Joint the frame is attached to
   */
  int parent_joint = 0;

  /**
   * @brief Parent frame
   */
  int parent_frame = 0;

  /**
   * @brief Placement of the frame relative to its parent joint
   */
  Eigen::Affine3d placement = Eigen::Affine3d::Identity();

  /**
   * @brief Frame type
   */
  Type type = OP_FRAME;

  /**
   * @brief Inertia attached to the frame (expressed in the frame)
   */
  Inertia inertia;
};

/**
 * @brief Kinematic tree of a robot: joints, bodies inertias, frames and limits. Joint 0 is the universe.
 */
class Model
{
public:
  Model();

  /**
   * @brief Model name
   */
  std::string name;

  /**
   * @brief Size of the configuration vector
   */
  int nq = 0;

  /**
   * @brief Number of degrees of freedom (size of the velocity vector)
   */
  int nv = 0;

  /**
   * @brief Number of joints (including the universe)
   */
  int njoints = 0;

  /**
   * @brief The joints
   */
  std::vector<Joint> joints;

  /**
   * @brief Joint names
   */
  std::vector<std::string> names;

  /**
   * @brief Parent of each joint
   */
  std::vector<int> parents;

  /**
   * @brief Placement of each joint in its parent joint frame (at the parent neutral configuration)
   */
  std::vector<Eigen::Affine3d> joint_placements;

  /**
   * @brief Inertia of the bodies attached to each joint, expressed in the joint frame
   */
  std::vector<Inertia> inertias;

  /**
   * @brief Joints from the universe (included) to each joint (included)
   */
  std::vector<std::vector<int>> supports;

  /**
   * @brief Joints of the subtree of each joint (including itself), in increasing order
   */
  std::vector<std::vector<int>> subtrees;

  /**
   * @brief Frames
   */
  std::vector<Frame> frames;

  /**
   * @brief Lower limits of the configuration
   */
  Eigen::VectorXd lower_position_limit;

  /**
   * @brief Upper limits of the configuration
   */
  Eigen::VectorXd upper_position_limit;

  /**
   * @brief Velocity limits (for each degree of freedom)
   */
  Eigen::VectorXd velocity_limit;

  /**
   * @brief Effort limits (for each degree of freedom)
   */
  Eigen::VectorXd effort_limit;

  /**
   * @brief Armature (for each degree of freedom): inertia added to the diagonal of the mass matrix
   */
  Eigen::VectorXd armature;

  /**
   * @brief Rotor inertia (for each degree of freedom)
   */
  Eigen::VectorXd rotor_inertia;

  /**
   * @brief Rotor gear ratio (for each degree of freedom)
   */
  Eigen::VectorXd rotor_gear_ratio;

  /**
   * @brief Gravity, in the world frame
   */
  Eigen::Vector3d gravity;

  /**
   * @brief Adds a joint to the tree
   * @param parent parent joint
   * @param joint joint
   * @param placement placement of the joint in its parent joint frame
   * @param name joint name
   * @param max_effort effort limits (nv)
   * @param max_velocity velocity limits (nv)
   * @param min_config lower configuration limits (nq)
   * @param max_config upper configuration limits (nq)
   * @return the index of the joint
   */
  int add_joint(int parent, const Joint& joint, const Eigen::Affine3d& placement, const std::string& name,
                const Eigen::VectorXd& max_effort, const Eigen::VectorXd& max_velocity,
                const Eigen::VectorXd& min_config, const Eigen::VectorXd& max_config);

  /**
   * @brief Adds a joint to the tree, without limits
   */
  int add_joint(int parent, const Joint& joint, const Eigen::Affine3d& placement, const std::string& name);

  /**
   * @brief Adds a frame. If a frame with the same name and type already exists, its index is returned.
   * @param frame the frame
   * @param append_inertia if true, the frame inertia is added to the inertia of its joint
   * @return frame index
   */
  int add_frame(const Frame& frame, bool append_inertia = true);

  /**
   * @brief Adds the frame of a joint (placed at the joint)
   */
  int add_joint_frame(int joint, int parent_frame);

  /**
   * @brief Adds the frame of a body attached to a joint
   */
  int add_body_frame(const std::string& name, int parent_joint, const Eigen::Affine3d& placement, int parent_frame);

  /**
   * @brief Adds the inertia of a body (expressed in a frame placed at placement in the joint frame) to the joint
   */
  void append_body_to_joint(int joint, const Inertia& inertia, const Eigen::Affine3d& placement);

  /**
   * @brief Checks if a frame exists
   */
  bool exist_frame(const std::string& name, int type = Frame::ANY) const;

  /**
   * @brief Index of the first frame with the given name (and type), or the number of frames if there is none
   */
  int get_frame_id(const std::string& name, int type = Frame::ANY) const;

  /**
   * @brief Checks if a joint exists
   */
  bool exist_joint(const std::string& name) const;

  /**
   * @brief Index of a joint (an error is raised if it doesn't exist)
   */
  int get_joint_id(const std::string& name) const;

  /**
   * @brief Neutral configuration
   */
  Eigen::VectorXd neutral() const;

  /**
   * @brief Integrates the configuration q with the velocity v over a unit of time
   */
  Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;

  /**
   * @brief Velocity that brings q0 to q1 in a unit of time
   */
  Eigen::VectorXd difference(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1) const;

  /**
   * @brief Random configuration, uniformly sampled within the configuration limits (uses rand())
   */
  Eigen::VectorXd random_configuration() const;

  /**
   * @brief Total mass of the bodies
   */
  double total_mass() const;
};
}  // namespace placo::model
