#include "placo/planning/lipm.h"

namespace placo
{
Eigen::VectorXd LIPM::Trajectory::com(double t)
{
  return Eigen::Vector2d(x.value(t, 0), y.value(t, 0));
}

Eigen::VectorXd LIPM::Trajectory::vel(double t)
{
  return Eigen::Vector2d(x.value(t, 1), y.value(t, 1));
}

Eigen::VectorXd LIPM::Trajectory::acc(double t)
{
  return omega_2 * (com(t) - zmp(t));
}

Eigen::VectorXd LIPM::Trajectory::zmp(double t)
{
  return Eigen::Vector2d(x.value(t, 2), y.value(t, 2));
}

Eigen::VectorXd LIPM::Trajectory::dzmp(double t)
{
  return Eigen::Vector2d(x.value(t, 3), y.value(t, 3));
}

Eigen::VectorXd LIPM::Trajectory::dcm(double t)
{
  return com(t) + (1 / omega) * vel(t);
}

LIPM::LIPM(Problem& problem, int timesteps, double omega, double dt, Eigen::Vector2d initial_pos,
           Eigen::Vector2d initial_vel, Eigen::Vector2d initial_zmp)
  : timesteps(timesteps), omega(omega), dt(dt)
{
  x_var = &problem.add_variable(timesteps);
  y_var = &problem.add_variable(timesteps);
  omega_2 = omega * omega;

  Eigen::MatrixXd M(4, 4);

  // Building a piecewize constant dZMP trajectory
  M << 0., 1., 0., 0.,            //
      omega_2, 0., -omega_2, 0.,  //
      0., 0., 0., 1.,             //
      0., 0., 0., 0.;             //

  x = Integrator(*x_var, Eigen::Vector3d(initial_pos.x(), initial_vel.x(), initial_zmp.x()), M, dt);
  y = Integrator(*y_var, Eigen::Vector3d(initial_pos.y(), initial_pos.y(), initial_pos.y()), M, dt);
}

Expression LIPM::com(int timestep)
{
  return x.expr(timestep, 0) / y.expr(timestep, 0);
}

Expression LIPM::vel(int timestep)
{
  return x.expr(timestep, 1) / y.expr(timestep, 1);
}

Expression LIPM::zmp(int timestep)
{
  return x.expr(timestep, 2) / y.expr(timestep, 2);
}

Expression LIPM::dzmp(int timestep)
{
  return x.expr(timestep, 3) / y.expr(timestep, 3);
}

Expression LIPM::acc(int timestep)
{
  return (x.expr(timestep, 0) * omega_2 - omega_2 * x.expr(timestep, 2)) /
         (y.expr(timestep, 0) * omega_2 - omega_2 * y.expr(timestep, 2));
}

Expression LIPM::dcm(int timestep)
{
  return (x.expr(timestep, 0) + (1 / omega) * x.expr(timestep, 1)) /
         (y.expr(timestep, 0) + (1 / omega) * y.expr(timestep, 1));
}

LIPM::Trajectory LIPM::get_trajectory()
{
  Trajectory trajectory;
  trajectory.omega = omega;
  trajectory.omega_2 = omega_2;

  trajectory.x = x.get_trajectory();
  trajectory.y = y.get_trajectory();

  return trajectory;
}

}  // namespace placo