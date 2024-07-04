#include "placo/tools/cubic_spline_3d.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace placo::tools
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

Eigen::Vector3d CubicSpline3D::pos(double t)
{
  return Eigen::Vector3d(xSpline.pos(t), ySpline.pos(t), zSpline.pos(t));
}

Eigen::Vector3d CubicSpline3D::vel(double t)
{
  return Eigen::Vector3d(xSpline.vel(t), ySpline.vel(t), zSpline.vel(t));
}

Eigen::Vector3d CubicSpline3D::acc(double t)
{
  return Eigen::Vector3d(xSpline.acc(t), ySpline.acc(t), zSpline.acc(t));
}

}  // namespace placo::tools