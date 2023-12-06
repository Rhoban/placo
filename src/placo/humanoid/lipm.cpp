#include <iostream>
#include "placo/humanoid/lipm.h"

namespace placo::humanoid
{
using namespace placo::problem;

Eigen::VectorXd LIPM::Trajectory::pos(double t)
{
  return Eigen::Vector2d(x.value(t, 0), y.value(t, 0));
}

Eigen::VectorXd LIPM::Trajectory::vel(double t)
{
  return Eigen::Vector2d(x.value(t, 1), y.value(t, 1));
}

Eigen::VectorXd LIPM::Trajectory::acc(double t)
{
  return Eigen::Vector2d(x.value(t, 2), y.value(t, 2));
}

Eigen::VectorXd LIPM::Trajectory::jerk(double t)
{
  return Eigen::Vector2d(x.value(t, 3), y.value(t, 3));
}

Eigen::VectorXd LIPM::Trajectory::dcm(double t, double omega)
{
  return pos(t) + (1 / omega) * vel(t);
}

Eigen::VectorXd LIPM::Trajectory::zmp(double t, double omega_2)
{
  return pos(t) - (1 / omega_2) * acc(t);
}

Eigen::VectorXd LIPM::Trajectory::dzmp(double t, double omega_2)
{
  return vel(t) + (1 / omega_2) * jerk(t);
}

LIPM::LIPM(Problem& problem, int timesteps, double dt, Eigen::Vector2d initial_pos, Eigen::Vector2d initial_vel,
           Eigen::Vector2d initial_acc)
  : timesteps(timesteps), dt(dt)
{
  x_var = &problem.add_variable(timesteps);
  y_var = &problem.add_variable(timesteps);

  x = Integrator(*x_var, Eigen::Vector3d(initial_pos.x(), initial_vel.x(), initial_acc.x()), 3, dt);
  y = Integrator(*y_var, Eigen::Vector3d(initial_pos.y(), initial_vel.y(), initial_acc.y()), 3, dt);
}

Expression LIPM::pos(int timestep)
{
  return x.expr(timestep, 0) / y.expr(timestep, 0);
}

Expression LIPM::vel(int timestep)
{
  return x.expr(timestep, 1) / y.expr(timestep, 1);
}

Expression LIPM::acc(int timestep)
{
  return x.expr(timestep, 2) / y.expr(timestep, 2);
}

Expression LIPM::jerk(int timestep)
{
  return x.expr(timestep, 3) / y.expr(timestep, 3);
}

Expression LIPM::dcm(int timestep, double omega)
{
  return (x.expr(timestep, 0) + (1 / omega) * x.expr(timestep, 1)) /
         (y.expr(timestep, 0) + (1 / omega) * y.expr(timestep, 1));
}

Expression LIPM::zmp(int timestep, double omega_2)
{
  return (x.expr(timestep, 0) - (1 / (omega_2)) * x.expr(timestep, 2)) /
         (y.expr(timestep, 0) - (1 / (omega_2)) * y.expr(timestep, 2));
}

Expression LIPM::dzmp(int timestep, double omega_2)
{
  return (x.expr(timestep, 1) - (1 / (omega_2)) * x.expr(timestep, 3)) /
         (y.expr(timestep, 1) - (1 / (omega_2)) * y.expr(timestep, 3));
}

double LIPM::compute_omega(double com_height)
{
  return sqrt(9.80665 / com_height);
}

LIPM::Trajectory LIPM::get_trajectory()
{
  Trajectory trajectory;

  trajectory.x = x.get_trajectory();
  trajectory.y = y.get_trajectory();

  trajectory.x.t_start = t_start;
  trajectory.y.t_start = t_start;

  return trajectory;
}

}  // namespace placo::humanoid