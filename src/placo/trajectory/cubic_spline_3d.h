#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include "placo/trajectory/cubic_spline.h"

namespace placo::trajectory
{
class CubicSpline3D
{
public:
  /**
   * Add a point with its x position, y value and its derivative slope
   */
  void add_point(double pos, Eigen::Vector3d val, Eigen::Vector3d delta);

  void clear();

  /**
   * Spline duration
   */
  double duration() const;

  /**
   * Return the spline interpolation
   * for given x position
   */
  Eigen::Vector3d pos(double x);

  /**
   * Returns the spline speed interpolation
   * for given x position
   */
  Eigen::Vector3d vel(double x);

private:
  CubicSpline xSpline;
  CubicSpline ySpline;
  CubicSpline zSpline;
};

}  // namespace placo::trajectory
