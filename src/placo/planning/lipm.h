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
    Eigen::VectorXd jerk(double t);

    Eigen::VectorXd dcm(double t, double omega);
    Eigen::VectorXd zmp(double t, double omega_2);
    Eigen::VectorXd dzmp(double t, double omega_2);

    Integrator::Trajectory x;
    Integrator::Trajectory y;
  };

  LIPM(Problem& problem, int timesteps, double dt, Eigen::Vector2d initial_pos,
       Eigen::Vector2d initial_vel = Eigen::Vector2d(0., 0.), Eigen::Vector2d initial_acc = Eigen::Vector2d(0., 0.));

  Trajectory get_trajectory();

  Expression pos(int timestep);
  Expression vel(int timestep);
  Expression acc(int timestep);
  Expression jerk(int timestep);

  Expression dcm(int timestep, double omega);
  Expression zmp(int timestep, double omega_2);
  Expression dzmp(int timestep, double omega_2);

  /**
   * @brief Compute the natural frequency of a LIPM given its height (omega = sqrt(g / h))
  */
  static double compute_omega(double com_height);

  // x and y integrators
  Integrator x;
  Integrator y;

  // Variables
  Variable* x_var;
  Variable* y_var;

  int timesteps;
  double dt;

  double t_start = 0.;
};
}  // namespace placo