#pragma once

#include <Eigen/Dense>
#include "placo/humanoid/humanoid_parameters.h"

namespace placo::humanoid
{
/**
 * @brief A quintic fitting of the swing foot
 */
class SwingFootQuintic
{
public:
  struct Trajectory
  {
    Eigen::Vector3d pos(double t);
    Eigen::Vector3d vel(double t);

    // Computed polynom (ax^5 + bx^4 + cx^3 + dx^2 + ex + f)
    Eigen::Vector3d a, b, c, d, e, f;

    double t_start, t_end;
  };

  static Trajectory make_trajectory(double t_start, double t_end, double height, Eigen::Vector3d start,
                                    Eigen::Vector3d target);
};
}  // namespace placo::humanoid