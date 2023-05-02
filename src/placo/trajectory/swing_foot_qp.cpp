#include "placo/trajectory/swing_foot_qp.h"
#include "eiquadprog/eiquadprog.hpp"

namespace placo
{
Eigen::Vector3d SwingFootQP::Trajectory::pos(double t)
{
  return Eigen::Vector3d(x.value(t, 0), y.value(t, 0), z.value(t, 0));
}

Eigen::Vector3d SwingFootQP::Trajectory::vel(double t)
{
  return Eigen::Vector3d(x.value(t, 1), y.value(t, 1), z.value(t, 1));
}

SwingFootQP::Trajectory SwingFootQP::make_trajectory(double t_start, double t_end, double min_height, double max_height,
                                                     Eigen::Vector3d start, Eigen::Vector3d target, int timesteps)
{
  double dt = (t_end - t_start) / timesteps;
  Problem problem;

  // x, y, z variables and integrators
  Variable& dddx = problem.add_variable(timesteps);
  Variable& dddy = problem.add_variable(timesteps);
  Variable& dddz = problem.add_variable(timesteps);
  Integrator x_integrator(dddx, Eigen::Vector3d(start.x(), 0., 0.), 3, dt);
  Integrator y_integrator(dddy, Eigen::Vector3d(start.y(), 0., 0.), 3, dt);
  Integrator z_integrator(dddz, Eigen::Vector3d(start.z(), 0., 0.), 3, dt);

  // Target conditions
  problem.add_constraint(x_integrator.expr(timesteps) == Eigen::Vector3d(target.x(), 0., 0.));
  problem.add_constraint(y_integrator.expr(timesteps) == Eigen::Vector3d(target.y(), 0., 0.));
  problem.add_constraint(z_integrator.expr(timesteps) == Eigen::Vector3d(target.z(), 0., 0.));

  // Z should reach a target height at 1/3 and 3/4 of the spline
  int k_start = timesteps / 6;
  int k_end = (timesteps * 5) / 6;

  for (int k = k_start; k <= k_end; k++)
  {
    problem.add_constraint(z_integrator.expr(k, 0) >= min_height);
    problem.add_constraint(z_integrator.expr(k, 0) <= max_height);
  }

  problem.solve();
  SwingFootQP::Trajectory trajectory;
  trajectory.x = x_integrator.get_trajectory();
  trajectory.y = y_integrator.get_trajectory();
  trajectory.z = z_integrator.get_trajectory();

  trajectory.x.t_start = t_start;
  trajectory.y.t_start = t_start;
  trajectory.z.t_start = t_start;

  return trajectory;
}
}  // namespace placo