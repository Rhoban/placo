#pragma once

#include "placo/problem/problem.h"
#include "placo/problem/integrator.h"
#include "placo/problem/constraints.h"

namespace placo
{
/**
 * @brief LIPM is an helper that can be used to build problem involving LIPM dynamics. The decision variables
 * introduced here are jerks, which is piecewise constant.
 */
class LIPM
{
public:
  struct Trajectory
  {
    Eigen::VectorXd pos(double t);
    Eigen::VectorXd vel(double t);
    Eigen::VectorXd acc(double t);
    Eigen::VectorXd zmp(double t);
    Eigen::VectorXd dcm(double t);
    Eigen::VectorXd dzmp(double t);
    Eigen::VectorXd jerk(double t);

    Integrator::Trajectory x;
    Integrator::Trajectory y;

    double omega;
    double omega_2;
  };

  LIPM(Problem& problem, int timesteps, double omega, double dt, Eigen::Vector2d initial_pos,
       Eigen::Vector2d initial_vel, Eigen::Vector2d initial_acc);

  Trajectory get_trajectory();

  Expression pos(int timestep);
  Expression vel(int timestep);
  Expression acc(int timestep);
  Expression zmp(int timestep);
  Expression dcm(int timestep);
  Expression dzmp(int timestep);
  Expression jerk(int timestep);

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

  double t_start = 0.;
};
}  // namespace placo