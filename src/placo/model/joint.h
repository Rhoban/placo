#pragma once

// PlaCo's rigid body model is a lighter, standalone implementation that uses pinocchio
// (https://github.com/stack-of-tasks/pinocchio) as its reference: it follows its conventions and formulas, so that
// the results are the same up to rounding errors.

#include <string>
#include <Eigen/Dense>
#include "placo/model/spatial.h"

namespace placo::model
{
/**
 * @brief A joint of the kinematic tree. Its configuration has nq coordinates (in the robot configuration q, starting
 * at idx_q) and its velocity nv coordinates (in the robot velocity, starting at idx_v)
 */
class Joint
{
public:
  enum Type
  {
    /**
     * @brief The universe (joint 0, no degree of freedom)
     */
    UNIVERSE = 0,

    /**
     * @brief Floating base: q = [x, y, z, qx, qy, qz, qw], velocity = [linear; angular], expressed in the joint
     * frame
     */
    FREE_FLYER = 1,

    /**
     * @brief Rotation about an axis, q is the angle
     */
    REVOLUTE = 2,

    /**
     * @brief Rotation about an axis without limits ("continuous"), q is [cos(angle), sin(angle)]
     */
    REVOLUTE_UNBOUNDED = 3,

    /**
     * @brief Translation along an axis
     */
    PRISMATIC = 4,

    /**
     * @brief Ball joint: q = [qx, qy, qz, qw], velocity is the angular velocity, expressed in the joint frame
     */
    SPHERICAL = 5
  };

  Joint(Type type = UNIVERSE, const Eigen::Vector3d& axis = Eigen::Vector3d::UnitX());

  /**
   * @brief Joint type
   */
  Type type;

  /**
   * @brief Rotation or translation axis (revolute and prismatic joints), unit vector
   */
  Eigen::Vector3d axis;

  /**
   * @brief Number of configuration coordinates
   */
  int nq;

  /**
   * @brief Number of degrees of freedom
   */
  int nv;

  /**
   * @brief Offset of the joint configuration in the robot configuration
   */
  int idx_q = -1;

  /**
   * @brief Offset of the joint velocity in the robot velocity
   */
  int idx_v = -1;

  /**
   * @brief Transformation of the joint frame (from the joint frame to its parent at its neutral configuration)
   * for the configuration q of the robot
   */
  Eigen::Affine3d transform(const Eigen::VectorXd& q) const;

  /**
   * @brief Motion subspace (6 x nv): spatial velocity of the joint frame (expressed in the joint frame) for each
   * degree of freedom
   */
  const Matrix6Xd& motion_subspace() const;

  /**
   * @brief Writes the neutral configuration of the joint in the robot configuration q
   */
  void neutral(Eigen::VectorXd& q) const;

  /**
   * @brief Integrates the joint configuration: q_out = q + v (in the joint configuration space)
   */
  void integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v, Eigen::VectorXd& q_out) const;

  /**
   * @brief Difference of two joint configurations: the velocity that brings q0 to q1 in a unit of time
   */
  void difference(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1, Eigen::VectorXd& d) const;

  /**
   * @brief Writes a random joint configuration (uniformly sampled within the limits) in q
   */
  void random(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, Eigen::VectorXd& q) const;

  /**
   * @brief Name of the joint type
   */
  std::string type_name() const;

protected:
  // Axis index for axes aligned with x, y or z (whose transformations are computed exactly), or -1
  int axis_index;

  // Motion subspace (see motion_subspace)
  Matrix6Xd S;

  // Rotation about the axis from the cosine and sine of the angle
  Eigen::Matrix3d axis_rotation(double c, double s) const;
};
}  // namespace placo::model
