#include "placo/kinematics/centroidal_momentum_task.h"
#include "placo/kinematics/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo::kinematics
{
CentroidalMomentumTask::CentroidalMomentumTask(Eigen::Vector3d L_world) : L_world(L_world)
{
}

void CentroidalMomentumTask::fill()
{
  Eigen::MatrixXd Ag = solver->robot.centroidal_map();
  Eigen::MatrixXd Ag_angular = Ag.block(3, 0, 3, solver->N);

  if (solver->dt == 0)
  {
    throw std::runtime_error("CentroidalMomentumTask: you should set solver.dt to use this task");
  }

  A.resize(mask.rows(), solver->N);
  b.resize(mask.rows(), 1);
  mask.apply(Ag_angular / solver->dt, A);
  mask.apply(L_world, b);
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