#include "placo/utils.h"
#include <iostream>

namespace placo {
Eigen::Affine3d interpolate_frames(Eigen::Affine3d frameA,
                                   Eigen::Affine3d frameB, double AtoB) {
  Eigen::Affine3d result;

  // Slerp for orientation
  Eigen::Quaterniond quaternionA(frameA.rotation());
  Eigen::Quaterniond quaternionB(frameB.rotation());
  result.linear() = quaternionA.slerp(AtoB, quaternionB).matrix();

  // Weighted average for translation
  result.translation() =
      frameA.translation() * (1 - AtoB) + frameB.translation() * AtoB;

  return result;
}

double frame_yaw(Eigen::Matrix3d rotation) {
  Eigen::Vector3d xInNewFrame = rotation * Eigen::Vector3d::UnitX();

  return atan2(xInNewFrame.y(), xInNewFrame.x());
}

Eigen::Affine3d frame(Eigen::Matrix4d matrix) {
  Eigen::Affine3d result;
  result.matrix() = matrix;

  return result;
}

Eigen::Affine3d flatten_on_floor(const Eigen::Affine3d &transformation) {
  Eigen::Affine3d flattened = transformation;

  // Setting z to 0
  flattened.translation().z() = 0;

  // Keeping only yaw
  flattened.linear() = Eigen::AngleAxisd(frame_yaw(transformation.rotation()),
                                         Eigen::Vector3d::UnitZ())
                           .toRotationMatrix();

  return flattened;
}

Eigen::Affine3d pin_se3_to_eigen(const pinocchio::GeometryData::SE3 &se3) {
  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translation() = se3.translation();
  transformation.linear() = se3.rotation();

  return transformation;
}
} // namespace placo