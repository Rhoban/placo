#include <cmath>
#include "placo/humanoid/humanoid_parameters.h"

namespace placo::humanoid
{
double HumanoidParameters::dt()
{
  return single_support_duration / single_support_timesteps;
}

int HumanoidParameters::double_support_timesteps()
{
  return std::round(double_support_ratio * single_support_timesteps);
}

int HumanoidParameters::startend_double_support_timesteps()
{
  return std::round(startend_double_support_ratio * single_support_timesteps);
}

double HumanoidParameters::double_support_duration()
{
  return double_support_timesteps() * dt();
}

double HumanoidParameters::startend_double_support_duration()
{
  return startend_double_support_timesteps() * dt();
}

bool HumanoidParameters::has_double_support()
{
  return double_support_timesteps() > 0;
}

Eigen::Vector3d HumanoidParameters::ellipsoid_clip(Eigen::Vector3d step)
{
  Eigen::Vector3d factor((step.x() >= 0) ? walk_max_dx_forward : walk_max_dx_backward, walk_max_dy, walk_max_dtheta);
  step.x() /= factor.x();
  step.y() /= factor.y();
  step.z() /= factor.z();

  double norm = step.norm();
  if (norm > 1)
  {
    step /= norm;
  }

  step.x() *= factor.x();
  step.y() *= factor.y();
  step.z() *= factor.z();

  return step;
}

Eigen::Affine3d HumanoidParameters::opposite_frame(HumanoidRobot::Side side, Eigen::Affine3d T_world_foot, double d_x,
                                                   double d_y, double d_theta)
{
  Eigen::Affine3d frame = T_world_foot;
  if (side == HumanoidRobot::Side::Left)
  {
    frame.translate(-feet_spacing * Eigen::Vector3d::UnitY());
  }
  else
  {
    frame.translate(feet_spacing * Eigen::Vector3d::UnitY());
  }

  frame.translate(Eigen::Vector3d(d_x, d_y, 0));
  frame.rotate(Eigen::AngleAxisd(d_theta, Eigen::Vector3d::UnitZ()));
  return frame;
}

Eigen::Affine3d HumanoidParameters::neutral_frame(HumanoidRobot::Side side, Eigen::Affine3d T_world_foot, double d_x,
                                                  double d_y, double d_theta)
{
  Eigen::Affine3d frame = T_world_foot;
  if (side == HumanoidRobot::Side::Left)
  {
    frame.translate(-(feet_spacing / 2.) * Eigen::Vector3d::UnitY());
  }
  else
  {
    frame.translate((feet_spacing / 2.) * Eigen::Vector3d::UnitY());
  }

  frame.translate(Eigen::Vector3d(d_x, d_y, 0));
  frame.rotate(Eigen::AngleAxisd(d_theta, Eigen::Vector3d::UnitZ()));
  return frame;
}
}  // namespace placo::humanoid