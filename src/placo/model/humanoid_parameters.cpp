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
}  // namespace placo