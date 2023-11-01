#include "placo/trajectory/cubic_spline_3d.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace placo::trajectory
{
void CubicSpline3D::clear()
{
  xSpline.clear();
  ySpline.clear();
  zSpline.clear();
}

double CubicSpline3D::duration() const
{
  return xSpline.duration();
}

void CubicSpline3D::add_point(double pos, Eigen::Vector3d val, Eigen::Vector3d delta)
{
  xSpline.add_point(pos, val[0], delta[0]);
  ySpline.add_point(pos, val[1], delta[1]);
  zSpline.add_point(pos, val[2], delta[2]);
}

Eigen::Vector3d CubicSpline3D::pos(double x)
{
  return Eigen::Vector3d(xSpline.pos(x), ySpline.pos(x), zSpline.pos(x));
}

Eigen::Vector3d CubicSpline3D::vel(double x)
{
  return Eigen::Vector3d(xSpline.vel(x), ySpline.vel(x), zSpline.vel(x));
}

}  // namespace placo::trajectory