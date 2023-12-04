#include "placo/humanoid/swing_foot_cubic.h"
#include "eiquadprog/eiquadprog.hpp"

namespace placo::humanoid
{

Eigen::Vector3d SwingFootCubic::Trajectory::pos(double t)
{
  return Eigen::Vector3d(x.pos(t), y.pos(t), z.pos(t));
}

Eigen::Vector3d SwingFootCubic::Trajectory::vel(double t)
{
  return Eigen::Vector3d(x.vel(t), y.vel(t), z.vel(t));
}

SwingFootCubic::Trajectory SwingFootCubic::make_trajectory(double t_start, double t_end, double height,
                                                           double rise_ratio, Eigen::Vector3d start,
                                                           Eigen::Vector3d target)
{
  SwingFootCubic::Trajectory trajectory;

  trajectory.x.add_point(t_start, start.x(), 0.);
  trajectory.y.add_point(t_start, start.y(), 0.);
  trajectory.z.add_point(t_start, start.z(), 0.);

  double half_duration = (t_end - t_start) / 2.;
  if (rise_ratio > 0)
  {
    trajectory.z.add_point(t_start + half_duration - half_duration * rise_ratio, height, 0.);
    trajectory.z.add_point(t_start + half_duration + half_duration * rise_ratio, height, 0.);
  }
  else
  {
    trajectory.z.add_point(t_start + half_duration, height, 0.);
  }

  trajectory.x.add_point(t_end, target.x(), 0.);
  trajectory.y.add_point(t_end, target.y(), 0.);
  trajectory.z.add_point(t_end, target.z(), 0.);

  return trajectory;
}

}  // namespace placo::humanoid