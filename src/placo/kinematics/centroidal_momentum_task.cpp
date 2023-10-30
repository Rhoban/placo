#include "placo/kinematics/centroidal_momentum_task.h"
#include "placo/kinematics/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo::kinematics
{
CentroidalMomentumTask::CentroidalMomentumTask(Eigen::Vector3d L_world) : L_world(L_world)
{
  active_axises.insert(0);
  active_axises.insert(1);
  active_axises.insert(2);
}

void CentroidalMomentumTask::mask_axis(int axis)
{
  active_axises.erase(axis);
}

void CentroidalMomentumTask::update()
{
  auto Ag = solver->robot.centroidal_map();

  A = Eigen::MatrixXd(active_axises.size(), solver->N);

  int k = 0;
  for (int axis : active_axises)
  {
    A.block(k, 0, 1, solver->N) = Ag.block(3 + axis, 0, 1, solver->N);
    k++;
  }

  b = L_world;
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