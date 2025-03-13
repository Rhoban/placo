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

SwingFootCubic::Trajectory SwingFootCubic::make_trajectory(double t_start, double virt_duration, double height,
  double rise_ratio, Eigen::Vector3d start, Eigen::Vector3d target, double elapsed_ratio, Eigen::Vector3d start_vel)
{
  SwingFootCubic::Trajectory trajectory;

  trajectory.x.add_point(t_start, start.x(), start_vel.x());
  trajectory.y.add_point(t_start, start.y(), start_vel.y());
  trajectory.z.add_point(t_start, start.z(), start_vel.z());

  double half_duration = virt_duration / 2.;
  double virt_t_start = t_start - elapsed_ratio * virt_duration;
  double t_end = t_start + (1 - elapsed_ratio) * virt_duration;

  std::vector<double> t;
  if (rise_ratio > 0)
  {
    t.push_back(virt_t_start + half_duration - half_duration * rise_ratio);
    t.push_back(virt_t_start + half_duration + half_duration * rise_ratio);
  }
  else
  {
    t.push_back(virt_t_start + half_duration);
  }

  for (double t_i : t)
  {
    if (t_i > t_start)
    {
      trajectory.z.add_point(t_i, height, 0.);
    }
  }

  trajectory.x.add_point(t_end, target.x(), 0.);
  trajectory.y.add_point(t_end, target.y(), 0.);
  trajectory.z.add_point(t_end, target.z(), 0.);

  return trajectory;
}

}  // namespace placo::humanoid