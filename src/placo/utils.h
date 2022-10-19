#pragma once

#include "pinocchio/spatial/se3.hpp"
#include <Eigen/Dense>

namespace placo {
/**
 * @brief Interpolate between two frames
 * @param frameA Frame A
 * @param frameB Frame B
 * @param AtoB A real number from 0 to 1 that controls the interpolation (0:
 * frame A, 1: frameB)
 * @return
 */
Eigen::Affine3d interpolate_frames(Eigen::Affine3d frameA,
                                   Eigen::Affine3d frameB, double AtoB);

/**
 * @brief Computes the "yaw" of an orientation
 * @param rotation the orientation
 * @return a scalar angle [rad]
 */
double frame_yaw(Eigen::Matrix3d rotation);
} // namespace placo
