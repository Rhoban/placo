#include "placo/utils.h"
#include <sys/stat.h>
#include <iostream>

namespace placo
{
Eigen::Affine3d interpolate_frames(Eigen::Affine3d frameA, Eigen::Affine3d frameB, double AtoB)
{
  Eigen::Affine3d result;

  // Slerp for orientation
  Eigen::Quaterniond quaternionA(frameA.rotation());
  Eigen::Quaterniond quaternionB(frameB.rotation());
  result.linear() = quaternionA.slerp(AtoB, quaternionB).matrix();

  // Weighted average for translation
  result.translation() = frameA.translation() * (1 - AtoB) + frameB.translation() * AtoB;

  return result;
}

double wrap_angle(double angle)
{
  return atan2(sin(angle), cos(angle));
}

double frame_yaw(Eigen::Matrix3d rotation)
{
  Eigen::Vector3d xInNewFrame = rotation * Eigen::Vector3d::UnitX();

  return atan2(xInNewFrame.y(), xInNewFrame.x());
}

Eigen::Affine3d frame(Eigen::Matrix4d matrix)
{
  Eigen::Affine3d result;
  result.matrix() = matrix;

  return result;
}

Eigen::Affine3d flatten_on_floor(const Eigen::Affine3d& transformation)
{
  Eigen::Affine3d flattened = transformation;

  // Setting z to 0
  flattened.translation().z() = 0;

  // Keeping only yaw
  flattened.linear() =
      Eigen::AngleAxisd(frame_yaw(transformation.rotation()), Eigen::Vector3d::UnitZ()).toRotationMatrix();

  return flattened;
}

Eigen::Affine3d pin_se3_to_eigen(const pinocchio::GeometryData::SE3& se3)
{
  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translation() = se3.translation();
  transformation.linear() = se3.rotation();

  return transformation;
}

double safe_acos(double v)
{
  if (v <= -1)
  {
    v = -1;
  }
  else if (v >= 1)
  {
    v = 1;
  }

  return acos(v);
}

bool file_exists(const std::string& name)
{
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

double velocity_limit(double torque, std::string dof, bool use_doc_limits)
{
  double min_velocity_limit = 0.1;

  // Fitting an affine function based on the plot from ROBOTIS for MX-64
  // https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/
  if (dof_to_motors[dof] == "mx_64")
  {
    return std::max(-1.4661 * std::abs(torque) + 6.9115, min_velocity_limit);
  }

  // Fitting a second order function based on the plot from ROBOTIS for MX-106
  // https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/
  if (use_doc_limits)
  {

  }
  // Or approximating the limit by an affine function
  return std::max(-0.79223 * std::abs(torque) + 4.5553, min_velocity_limit);
}

std::map<std::string, std::string> dof_to_motors = {
  { "left_hip_yaw", "mx_64" },
  { "right_hip_yaw", "mx_64" },
  { "left_hip_pitch", "mx_106" },
  { "right_hip_pitch", "mx_106" },
  { "left_hip_roll", "mx_106" },
  { "right_hip_roll", "mx_106" },
  { "left_knee", "mx_106" },
  { "right_knee", "mx_106" },
  { "left_ankle_pitch", "mx_106" },
  { "right_ankle_pitch", "mx_106" },
  { "left_ankle_roll", "mx_106" },
  { "right_ankle_roll", "mx_106" },
  { "left_shoulder_pitch", "mx_64" },
  { "right_shoulder_pitch", "mx_64" },
  { "left_shoulder_roll", "mx_64" },
  { "right_shoulder_roll", "mx_64" },
  { "left_elbow", "mx_64" },
  { "right_elbow", "mx_64" },
  { "head_yaw", "mx_64" },
  { "head_pitch", "mx_64" }
};
}  // namespace placo