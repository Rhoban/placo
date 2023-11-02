#include "placo/dynamics/task.h"

namespace placo::dynamics
{
double Task::get_kd()
{
  if (critically_damped)
  {
    return 2. * sqrt(kp);
  }
  else
  {
    return kd;
  }
}
}  // namespace placo::dynamics