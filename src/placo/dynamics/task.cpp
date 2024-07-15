#include "placo/dynamics/task.h"

namespace placo::dynamics
{
double Task::get_kd()
{
  if (kd < 0.0)
  {
    return 2. * sqrt(kp);
  }
  else
  {
    return kd;
  }
}
}  // namespace placo::dynamics