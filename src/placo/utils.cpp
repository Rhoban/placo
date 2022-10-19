#include "placo/utils.h"

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
} // namespace placo