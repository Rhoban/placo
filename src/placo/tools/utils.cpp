#include "placo/tools/utils.h"
#include <sys/stat.h>
#include <iostream>

namespace placo::tools
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

Eigen::Matrix3d rotation_from_axis(std::string axis, Eigen::Vector3d vector)
{
  Eigen::Matrix3d R;
  vector.normalize();

  Eigen::Vector3d vector_id;
  if (axis == "x")
  {
    vector_id = Eigen::Vector3d::UnitX();
  }
  else if (axis == "y")
  {
    vector_id = Eigen::Vector3d::UnitY();
  }
  else if (axis == "z")
  {
    vector_id = Eigen::Vector3d::UnitZ();
  }
  else
  {
    throw std::runtime_error("Unknown axis: " + axis);
  }

  Eigen::Vector3d w = vector_id.cross(vector);
  double theta = safe_acos(vector_id.dot(vector));
  if (w.norm() == 0)
  {
    if (axis == "x")
      w = Eigen::Vector3d::UnitY();
    if (axis == "y")
      w = Eigen::Vector3d::UnitZ();
    if (axis == "z")
      w = Eigen::Vector3d::UnitX();
  }
  else
  {
    w.normalize();
  }

  return pinocchio::exp3(w * theta);
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

Eigen::Affine3d optimal_transformation(Eigen::MatrixXd points_in_A, Eigen::MatrixXd points_in_B)
{
  if (points_in_A.cols() != 3 || points_in_B.cols() != 3)
  {
    throw std::runtime_error("optimal_transformation(): points should have 3 columns (x, y, z)");
  }
  if (points_in_A.rows() != points_in_B.rows())
  {
    throw std::runtime_error("optimal_transformation(): points in A and B should have the same number of rows");
  }
  if (points_in_A.rows() < 3)
  {
    throw std::runtime_error("optimal_transformation(): at least 3 points are required");
  }

  Eigen::Affine3d T_a_b = Eigen::Affine3d::Identity();

  // Compute the barycenters
  Eigen::Vector3d barycenter_A = points_in_A.colwise().mean();
  Eigen::Vector3d barycenter_B = points_in_B.colwise().mean();

  // Compute the centered points
  Eigen::MatrixXd centered_A = points_in_A.rowwise() - barycenter_A.transpose();
  Eigen::MatrixXd centered_B = points_in_B.rowwise() - barycenter_B.transpose();

  // Compute the covariance matrix
  Eigen::Matrix3d H = centered_B.transpose() * centered_A;

  // Compute the SVD
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // Compute the rotation
  Eigen::Matrix3d R = V * U.transpose();

  if (R.determinant() < 0)
  {
    Eigen::Matrix3d Z = Eigen::Matrix3d::Identity();
    Z(2, 2) = -1;
    R = V * Z * U.transpose();
  }

  // Compute the translation
  Eigen::Vector3d t = barycenter_A - R * barycenter_B;

  T_a_b.linear() = R;
  T_a_b.translation() = t;

  return T_a_b;
}
}  // namespace placo::tools