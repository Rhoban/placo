#pragma once

#include <Eigen/Dense>

namespace placo
{
  /**
   * Computes the average between two given frames
   */
  Eigen::Affine3d average_frames(Eigen::Affine3d frameA, Eigen::Affine3d frameB, double AtoB);

  /**
   * Computes the "yaw" of a frame
   */
  double frame_yaw(Eigen::Matrix3d rotation);
}

