#pragma once

// PlaCo's rigid body model is a lighter, standalone implementation that uses pinocchio
// (https://github.com/stack-of-tasks/pinocchio) as its reference: it follows its conventions and formulas, so that
// the results are the same up to rounding errors.

#include <Eigen/Dense>

namespace placo::model
{
/**
 * @brief Spatial vector (velocity or force), stored as [linear; angular]
 */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief 6x6 matrix (e.g spatial inertia or action matrix)
 */
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

/**
 * @brief Matrix with 6 rows (e.g a Jacobian, whose columns are spatial velocities)
 */
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;

/**
 * @brief Reference frame in which a Jacobian (or a spatial velocity) is expressed
 */
enum ReferenceFrame
{
  /**
   * @brief Velocity of the point that coincides with the world origin, expressed in the world frame
   */
  WORLD = 0,

  /**
   * @brief Velocity of the frame origin, expressed in the frame
   */
  LOCAL = 1,

  /**
   * @brief Velocity of the frame origin, expressed in the world frame axes
   */
  LOCAL_WORLD_ALIGNED = 2
};

/**
 * @brief Skew-symmetric matrix of a vector, such that skew(a) * b = a x b
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& v);

/**
 * @brief Inverse of a rigid transformation (rotation and translation)
 */
Eigen::Affine3d inverse(const Eigen::Affine3d& T);

/**
 * @brief Expresses a spatial velocity given in frame b in frame a, T being the transformation from b to a
 */
Vector6d act(const Eigen::Affine3d& T, const Vector6d& v);

/**
 * @brief Expresses a spatial velocity given in frame a in frame b, T being the transformation from b to a
 */
Vector6d act_inv(const Eigen::Affine3d& T, const Vector6d& v);

/**
 * @brief Expresses a spatial force given in frame b in frame a, T being the transformation from b to a
 */
Vector6d act_force(const Eigen::Affine3d& T, const Vector6d& f);

/**
 * @brief Cross product of two spatial velocities (the time derivative of v2 when it moves with v1)
 */
Vector6d motion_cross(const Vector6d& v1, const Vector6d& v2);

/**
 * @brief Cross product of a spatial velocity and a spatial force
 */
Vector6d force_cross(const Vector6d& v, const Vector6d& f);

/**
 * @brief 6x6 matrix expressing spatial velocities given in frame b in frame a, T being the transformation
 * from b to a
 */
Matrix6d action_matrix(const Eigen::Affine3d& T);

/**
 * @brief Exponential map of SO(3): rotation matrix of a rotation vector (axis times angle)
 */
Eigen::Matrix3d exp3(const Eigen::Vector3d& v);

/**
 * @brief Logarithm map of SO(3): rotation vector (axis times angle) of a rotation matrix
 * @param R rotation matrix
 * @param theta (output) the rotation angle, in [0, pi]
 */
Eigen::Vector3d log3(const Eigen::Matrix3d& R, double& theta);

/**
 * @brief Logarithm map of SO(3): rotation vector (axis times angle) of a rotation matrix
 */
Eigen::Vector3d log3(const Eigen::Matrix3d& R);

/**
 * @brief Jacobian of the logarithm map of SO(3)
 */
Eigen::Matrix3d jlog3(const Eigen::Matrix3d& R);

/**
 * @brief Exponential map of SO(3), as a unit quaternion
 */
Eigen::Quaterniond quaternion_exp3(const Eigen::Vector3d& v);

/**
 * @brief Logarithm map of SO(3), from a unit quaternion
 * @param quaternion unit quaternion
 * @param theta (output) the rotation angle, in [0, pi]
 */
Eigen::Vector3d quaternion_log3(const Eigen::Quaterniond& quaternion, double& theta);

/**
 * @brief Exponential map of SE(3): integrates a spatial velocity [linear; angular] (expressed in the moving frame)
 * over a unit of time
 * @param v spatial velocity
 * @param translation (output) translation
 * @param quaternion (output) rotation
 */
void exp6(const Vector6d& v, Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion);

/**
 * @brief Exponential map of SE(3): integrates a spatial velocity [linear; angular] (expressed in the moving frame)
 * over a unit of time
 * @param v spatial velocity
 * @return the transformation
 */
Eigen::Affine3d exp6(const Vector6d& v);

/**
 * @brief Logarithm map of SE(3)
 * @param T transformation
 * @return the spatial velocity [linear; angular] that, integrated over a unit of time, yields the transformation
 */
Vector6d log6(const Eigen::Affine3d& T);

/**
 * @brief Logarithm map of SE(3), from a translation and a unit quaternion
 * @return the spatial velocity [linear; angular] that, integrated over a unit of time, yields the transformation
 */
Vector6d log6(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& translation);

/**
 * @brief Normalizes a unit quaternion that drifted away from the unit norm (first order approximation)
 */
void first_order_normalize(Eigen::Quaterniond& quaternion);

/**
 * @brief Rotation matrix from roll, pitch and yaw: R = Rz(yaw) Ry(pitch) Rx(roll)
 */
Eigen::Matrix3d rpy_to_matrix(double roll, double pitch, double yaw);

/**
 * @brief Roll, pitch and yaw of a rotation matrix (R = Rz(yaw) Ry(pitch) Rx(roll)), with roll and yaw in
 * [-pi, pi] and pitch in [-pi/2, pi/2]
 */
Eigen::Vector3d matrix_to_rpy(const Eigen::Matrix3d& R);

/**
 * @brief Inertia of a rigid body: mass, center of mass and rotational inertia
 */
struct Inertia
{
  /**
   * @brief Mass [kg]
   */
  double mass = 0.;

  /**
   * @brief Center of mass, in the body frame
   */
  Eigen::Vector3d lever = Eigen::Vector3d::Zero();

  /**
   * @brief Rotational inertia about the center of mass, in the body frame axes
   */
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();

  Inertia() = default;
  Inertia(double mass, const Eigen::Vector3d& lever, const Eigen::Matrix3d& inertia);

  /**
   * @brief True if the mass, center of mass and rotational inertia are all exactly zero
   */
  bool is_zero() const;

  /**
   * @brief The same inertia, expressed in frame a, T being the transformation from the body frame to a
   */
  Inertia transformed(const Eigen::Affine3d& T) const;

  /**
   * @brief Adds another inertia (expressed in the same frame), as if the two bodies were rigidly attached
   */
  Inertia& operator+=(const Inertia& other);

  /**
   * @brief 6x6 spatial inertia matrix, mapping a spatial velocity to a spatial momentum
   */
  Matrix6d matrix() const;

  /**
   * @brief Spatial momentum of the body for the given spatial velocity
   */
  Vector6d momentum(const Vector6d& v) const;
};
}  // namespace placo::model
