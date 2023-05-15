#pragma once

#include <Eigen/Dense>
#include "placo/model/humanoid_parameters.h"
#include "placo/trajectory/foot_trajectory.h"

namespace placo
{
/**
 * @brief A cubic fitting of swing foot, see:
 * https://scaron.info/doc/pymanoid/walking-pattern-generation.html#pymanoid.swing_foot.SwingFoot
 */
class SwingFoot
{
public:
  struct SwingTrajectory : FootTrajectory
  {
    Eigen::Vector3d pos(double t);
    Eigen::Vector3d vel(double t);

    // Computed polynom (ax^3 + bx^2 + cx + d)
    Eigen::Vector3d a, b, c, d;
  };

  static SwingTrajectory make_trajectory(double t_start, double t_end, double height, Eigen::Vector3d start,
                                         Eigen::Vector3d target);

  static SwingTrajectory remake_trajectory(SwingTrajectory& old_trajectory, double t, Eigen::Vector3d target);
};
}  // namespace placo