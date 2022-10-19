#include "placo/utils.h"

namespace placo
{
Eigen::Affine3d average_frames(Eigen::Affine3d frameA, Eigen::Affine3d frameB, double AtoB)
{
  // Slerp for orientation
  Eigen::Quaterniond quaternionA(frameA.rotation());
  Eigen::Quaterniond quaternionB(frameB.rotation());
  Eigen::Quaterniond q = quaternionA.slerp(AtoB, quaternionB);

  // Weighted average for translation
  Eigen::Vector3d translationA(frameA.translation().x(), frameA.translation().y(), frameA.translation().z());
  Eigen::Vector3d translationB(frameB.translation().x(), frameB.translation().y(), frameB.translation().z());
  Eigen::Vector3d t = translationA * (1 - AtoB) + translationB * AtoB;

  Eigen::Affine3d result;
  result.fromPositionOrientationScale(t, q, Eigen::Vector3d(1, 1, 1));

  return result;
}

double frame_yaw(Eigen::Matrix3d rotation)
{
  Eigen::Vector3d xInNewFrame = rotation * Eigen::Vector3d::UnitX();

  return atan2(xInNewFrame.y(), xInNewFrame.x());
}
}