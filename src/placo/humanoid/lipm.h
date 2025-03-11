#pragma once

#include "placo/problem/problem.h"
#include "placo/problem/integrator.h"

namespace placo::humanoid
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

    problem::Integrator::Trajectory x;
    problem::Integrator::Trajectory y;
  };

  LIPM();
  LIPM(problem::Problem& problem, double dt, int timesteps, double t_start, LIPM& previous);
  LIPM(problem::Problem& problem, double dt, int timesteps, double t_start, Eigen::Vector2d initial_pos, 
       Eigen::Vector2d initial_vel = Eigen::Vector2d::Zero(), Eigen::Vector2d initial_acc = Eigen::Vector2d::Zero());

  // Build a LIPM from a previous one. Necessary for python bindings.
  static LIPM build_LIPM_from_previous(problem::Problem& problem, double dt, int timesteps, double t_start, LIPM& previous);

  // Timestep duration
  double dt;

  // Number of timesteps
  int timesteps;

  // Time at the start of the LIPM trajectory
  double t_start;

  // Time at the end of the LIPM trajectory
  double t_end();
  
  problem::Expression pos(int timestep);
  problem::Expression vel(int timestep);
  problem::Expression acc(int timestep);
  problem::Expression jerk(int timestep);

  problem::Expression dcm(int timestep, double omega);
  problem::Expression zmp(int timestep, double omega_2);
  problem::Expression dzmp(int timestep, double omega_2);

  // Integrators
  problem::Integrator x;
  problem::Integrator y;

  // Variables
  problem::Variable* x_var;
  problem::Variable* y_var;

  /**
   * @brief Get the LIPM trajectory. Should be used after solving the problem.
   */
  Trajectory get_trajectory();

  /**
   * @brief Compute the natural frequency of a LIPM given its height (omega = sqrt(g / h))
   */
  static double compute_omega(double com_height);
};
}  // namespace placo::humanoid