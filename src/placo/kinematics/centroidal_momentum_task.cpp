#include "placo/kinematics/centroidal_momentum_task.h"
#include "placo/kinematics/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo::kinematics
{
CentroidalMomentumTask::CentroidalMomentumTask(Eigen::Vector3d L_world) : L_world(L_world)
{
}

void CentroidalMomentumTask::update()
{
  auto Ag = solver->robot.centroidal_map();

  A = mask.apply(Ag);
  b = mask.apply(L_world);
}

std::string CentroidalMomentumTask::type_name()
{
  return "centroidal_momentum";
}

std::string CentroidalMomentumTask::error_unit()
{
  return "N.m rad/s";
}
}  // namespace placo::kinematics