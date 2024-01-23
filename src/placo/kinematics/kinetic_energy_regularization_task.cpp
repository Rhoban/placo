#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "placo/kinematics/kinetic_energy_regularization_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void KineticEnergyRegularizationTask::update()
{
  // Computing the mass matrix
  Eigen::MatrixXd M = solver->robot.mass_matrix();

  // Computing M^1/2
  Eigen::MatrixXd M_1_2 = M.sqrt();

  // We need dt so that this task has an energy unit
  if (solver->dt == 0.)
  {
    throw std::runtime_error("RegularizationTask::update: you should set solver.dt");
  }

  // We want to minimize (1/2) * qd^T * M * qd
  // Equality equation is M^1/2 / (dt * sqrt(2)) * delta_q = 0
  A = M_1_2 / (sqrt(2) * solver->dt);
  b = Eigen::VectorXd(solver->N);
  b.setZero();
}

std::string KineticEnergyRegularizationTask::type_name()
{
  return "kinetic_energy_regularization";
}

std::string KineticEnergyRegularizationTask::error_unit()
{
  return "joule";
}
}  // namespace placo::kinematics