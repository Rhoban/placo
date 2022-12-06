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
}  // namespace placo