#include "placo/kinematics/com_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
CoMTask::CoMTask(Eigen::Vector3d target_world) : target_world(target_world)
{
}

void CoMTask::fill()
{
  A.resize(mask.rows(), solver->N);
  b.resize(mask.rows(), 1);
  mask.apply(solver->robot.com_jacobian(), A);
  mask.apply(target_world - solver->robot.com_world(), b);
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