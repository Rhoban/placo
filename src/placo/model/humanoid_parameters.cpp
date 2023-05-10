#include <cmath>
#include "placo/model/humanoid_parameters.h"

namespace placo
{
double HumanoidParameters::omega()
{
  if (pendulum_height > 0.)
  {
    return sqrt(9.80665 / pendulum_height);
  }
  else
  {
    return 0.;
  }
}

double HumanoidParameters::dt()
{
  return single_support_duration / ((double)single_support_timesteps);
}

double HumanoidParameters::double_support_duration()
{
  return double_support_ratio * single_support_duration;
}

double HumanoidParameters::startend_double_support_duration()
{
  return startend_double_support_ratio * single_support_duration;
}

int HumanoidParameters::double_support_timesteps()
{
  return std::round(double_support_ratio * single_support_timesteps);
}

int HumanoidParameters::startend_double_support_timesteps()
{
  return std::round(startend_double_support_ratio * single_support_timesteps);
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
}  // namespace placo