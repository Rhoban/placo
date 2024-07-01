#pragma once

#include "pinocchio/multibody/geometry.hpp"
#include <Eigen/Dense>

namespace placo::tools
{
/**
 * @brief Interpolate between two frames
 * @param frameA Frame A
 * @param frameB Frame B
 * @param AtoB A real number from 0 to 1 that controls the interpolation (0: frame A, 1: frameB)
 * @return
 */
Eigen::Affine3d interpolate_frames(Eigen::Affine3d frameA, Eigen::Affine3d frameB, double AtoB);

/**
 * @brief Wraps an angle between -pi and pi
 */
double wrap_angle(double angle);

/**
 * @brief Computes the "yaw" of an orientation
 * @param rotation the orientation
 * @return a scalar angle [rad]
 */
double frame_yaw(Eigen::Matrix3d rotation);

/**
 * @brief Builds a rotation matrix with a given axis target
 * @param axis axis (x, y or z)
 * @param vector target (unit) vector
 * @return 3x3 rotation matrix
 */
Eigen::Matrix3d rotation_from_axis(std::string axis, Eigen::Vector3d vector);

/**
 * @brief Takes a 3D transformation and ensure it is "flat" on the floor
 * (setting z to 0 and keeping only yaw)
 * @param transformation a 3D transformation
 * @return a 3D transformation that lies on the floor (no pitch/roll and no z)
 */
Eigen::Affine3d flatten_on_floor(const Eigen::Affine3d& transformation);

/**
 * @brief Converts a pinocchio's SE3 transformation to Eigen Affine3d
 * @param se3 pinocchio SE3 transformation
 * @return Eigen affine3d
 */
Eigen::Affine3d pin_se3_to_eigen(const pinocchio::GeometryData::SE3& se3);

/**
 * @brief Returns the acos of v, with safety on v values
 * @param v value
 */
double safe_acos(double v);

/**
 * @brief Check file existence
 */
bool file_exists(const std::string& name);

/**
 * @brief Finds the optimal transformation T_a_b that minimizes the sum of squared distances
 * between the (same) points with coordinates expressed in A and B.
 * Points are stacked in lines (columns are x, y and z) in the matrices
 */
Eigen::Affine3d optimal_transformation(Eigen::MatrixXd points_in_A, Eigen::MatrixXd points_in_B);

}  // namespace placo::tools
