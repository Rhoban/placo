#pragma once

#include "placo/problem/problem.h"
#include "placo/problem/integrator.h"
#include "placo/problem/constraints.h"

namespace placo
{
class LIPM
{
public:
  struct Trajectory
  {
    Eigen::VectorXd com(double t);
    Eigen::VectorXd vel(double t);
    Eigen::VectorXd acc(double t);
    Eigen::VectorXd zmp(double t);
    Eigen::VectorXd dcm(double t);

    Integrator::Trajectory x;
    Integrator::Trajectory y;

    double omega;
    double omega_2;
  };

  LIPM(int timesteps, double omega, double dt, Eigen::Vector2d initial_pos, Eigen::Vector2d initial_vel,
       Eigen::Vector2d initial_zmp);

  Trajectory get_trajectory();

  Expression com(int timestep);
  Expression vel(int timestep);
  Expression acc(int timestep);
  Expression zmp(int timestep);
  Expression dcm(int timestep);

  // Problem
  Problem problem;

  // x and y integrators
  Integrator x;
  Integrator y;

  // Variables
  Variable* x_var;
  Variable* y_var;

  int timesteps;
  double omega;
  double omega_2;
  double dt;
};
}  // namespace placo