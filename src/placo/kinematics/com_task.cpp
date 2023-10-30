#include "placo/kinematics/com_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
CoMTask::CoMTask(Eigen::Vector3d target_world) : target_world(target_world)
{
}

void CoMTask::update()
{
  A = solver->robot.com_jacobian()(mask.indices, Eigen::placeholders::all);
  b = (target_world - solver->robot.com_world())(mask.indices, Eigen::placeholders::all);
}

std::string CoMTask::type_name()
{
  return "com";
}

std::string CoMTask::error_unit()
{
  return "m";
}
}  // namespace placo::kinematics