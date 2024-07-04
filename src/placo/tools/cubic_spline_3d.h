#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include "placo/tools/cubic_spline.h"

namespace placo::tools
{
class CubicSpline3D
{
public:
  /**
   * @brief Adds a point
   * @param t time
   * @param x value (3D vector)
   * @param dx velocity (3D vector)
   */
  void add_point(double t, Eigen::Vector3d x, Eigen::Vector3d dx);

  /**
   * @brief Clears the spline
   */
  void clear();

  /**
   * @brief Spline duration
   * @return spline duration in seconds
   */
  double duration() const;

  /**
   * @brief Returns the spline value at time t
   * @param t time
   * @return position (3D vector)
   */
  Eigen::Vector3d pos(double t);

  /**
   * @brief Returns the spline velocity at time t
   * @param t time
   * @return velocity (3D vector)
   */
  Eigen::Vector3d vel(double t);

  /**
   * @brief Returns the spline accleeration at time t
   * @param t time
   * @return acceleration (3D vector)
   */
  Eigen::Vector3d acc(double t);

private:
  CubicSpline xSpline;
  CubicSpline ySpline;
  CubicSpline zSpline;
};

}  // namespace placo::tools
