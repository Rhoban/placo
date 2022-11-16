#include "placo/control/com_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
CoMTask::CoMTask(Eigen::Vector3d target_world) : target_world(target_world)
{
}

void CoMTask::update()
{
  A = solver->robot->com_jacobian();
  b = target_world - solver->robot->com_world();
}

std::string CoMTask::type_name()
{
  return "com";
}

std::string CoMTask::error_unit()
{
  return "m";
}
}  // namespace placo