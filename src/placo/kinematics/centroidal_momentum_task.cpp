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
  Eigen::MatrixXd Ag = solver->robot.centroidal_map();
  Eigen::MatrixXd Ag_angular = Ag.block(3, 0, 3, solver->N);

  A = mask.apply(Ag_angular);
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