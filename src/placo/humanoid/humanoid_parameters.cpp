#include <cmath>
#include "placo/humanoid/humanoid_parameters.h"

namespace placo::humanoid
{
double HumanoidParameters::dt()
{
  return single_support_duration / ((double)single_support_timesteps);
}

double HumanoidParameters::double_support_duration()
{
  return ratio_duration(double_support_ratio);
}

double HumanoidParameters::startend_double_support_duration()
{
  return ratio_duration(startend_double_support_ratio);
}

int HumanoidParameters::double_support_timesteps()
{
  return ratio_timesteps(double_support_ratio);
}

int HumanoidParameters::startend_double_support_timesteps()
{
  return ratio_timesteps(startend_double_support_ratio);
}

double HumanoidParameters::kick_up_duration()
{
  return ratio_duration(kick_ratio_up);
}

double HumanoidParameters::kick_shot_duration()
{
  return ratio_duration(kick_ratio_shot);
}

double HumanoidParameters::kick_neutral_duration()
{
  return ratio_duration(kick_ratio_neutral);
}

double HumanoidParameters::kick_down_duration()
{
  return ratio_duration(kick_ratio_down);
}

double HumanoidParameters::kick_support_ratio()
{
  return kick_ratio_up + kick_ratio_shot + kick_ratio_neutral + kick_ratio_down;
}

double HumanoidParameters::kick_support_duration()
{
  return ratio_duration(kick_support_ratio());
}

int HumanoidParameters::kick_support_timesteps()
{
  return ratio_timesteps(kick_support_ratio());
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

int HumanoidParameters::ratio_timesteps(double ratio)
{
  return std::round(ratio * single_support_timesteps);
}

double HumanoidParameters::ratio_duration(double ratio)
{
  return ratio_timesteps(ratio) * dt();
}
}  // namespace placo::humanoid