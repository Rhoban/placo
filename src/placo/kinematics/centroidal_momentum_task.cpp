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

  if (solver->dt == 0)
  {
    throw std::runtime_error("CentroidalMomentumTask: you should set solver.dt to use this task");
  }

  A = mask.apply(Ag_angular) / solver->dt;
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