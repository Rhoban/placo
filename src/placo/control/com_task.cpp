#include "placo/control/com_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
CoMTask::CoMTask(Eigen::Vector3d target_world) : target_world(target_world)
{
}

void CoMTask::update()
{
  std::vector<int> indices;
  if (mask & MaskX)
  {
    indices.push_back(0);
  }
  if (mask & MaskY)
  {
    indices.push_back(1);
  }
  if (mask & MaskZ)
  {
    indices.push_back(2);
  }

  A = solver->robot->com_jacobian()(indices, Eigen::placeholders::all);
  b = (target_world - solver->robot->com_world())(indices, Eigen::placeholders::all);
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