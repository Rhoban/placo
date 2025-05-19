#include "placo/tools/cubic_spline.h"
#include "placo/tools/utils.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace placo::tools
{
CubicSpline::CubicSpline(bool angular) : angular(angular)
{
}

void CubicSpline::add_point(double t, double x, double dx)
{
  // Discontinuity of angle
  if (angular && _points.size() > 0)
  {
    x = _points.back().x + tools::wrap_angle(x - _points.back().x);
  }
  struct Point point = { t, x, dx };

  if (_points.size() > 0 && t <= _points.back().t)
  {
    throw std::runtime_error("Trying to add a point in a cublic spline before a previous one");
  }

  _points.push_back(point);
  dirty = true;
}

void CubicSpline::clear()
{
  _points.clear();
  _splines.clear();
}

double CubicSpline::duration() const
{
  return _splines[_splines.size() - 1].t_end - _splines[0].t_start;
}

/**
 * Return the spline interpolation
 * for given x position
 */
double CubicSpline::interpolation(double t, CubicSpline::ValueType type)
{
  if (dirty)
  {
    compute_splines();
    dirty = false;
  }

  if (_points.size() == 0)
  {
    return 0.0;
  }
  else if (_points.size() == 1)
  {
    if (type == Value)
    {
      return _points.front().x;
    }
    else
    {
      return _points.front().dx;
    }
  }
  else
  {
    if (t < _splines.front().t_start)
    {
      t = _splines.front().t_start;
    }
    if (t > _splines.back().t_end)
    {
      t = _splines.back().t_end;
    }

    for (size_t i = 0; i < _splines.size(); i++)
    {
      if (t >= _splines[i].t_start && t <= _splines[i].t_end)
      {
        if (type == Value)
        {
          return polynom_value(t - _splines[i].t_start, _splines[i].poly);
        }
        else if (type == Speed)
        {
          return polynom_diff(t - _splines[i].t_start, _splines[i].poly);
        }
        else if (type == Acceleration)
        {
          return polynom_diff2(t - _splines[i].t_start, _splines[i].poly);
        }
      }
    }
    return 0.0;
  }
}

double CubicSpline::pos(double t)
{
  return interpolation(t, Value);
}

double CubicSpline::vel(double t)
{
  return interpolation(t, Speed);
}

double CubicSpline::acc(double t)
{
  return interpolation(t, Acceleration);
}

/**
 * Access to internal Points container
 */
const CubicSpline::Points& CubicSpline::points() const
{
  return _points;
}

double CubicSpline::polynom_value(double t, const Polynom& p)
{
  return p.d + t * (t * (p.a * t + p.b) + p.c);
}

double CubicSpline::polynom_diff(double t, const Polynom& p)
{
  return t * (3 * p.a * t + 2 * p.b) + p.c;
}

double CubicSpline::polynom_diff2(double t, const Polynom& p)
{
  return 6 * p.a * t + 2 * p.b;
}

CubicSpline::Polynom CubicSpline::fit(double t1, double x1, double dx1, double t2, double x2, double dx2)
{
  Eigen::Matrix4d M;
  double t1_2 = t1 * t1;
  double t1_3 = t1_2 * t1;
  double t2_2 = t2 * t2;
  double t2_3 = t2_2 * t2;

  M << t1_3, t1_2, t1, 1,      //
      3 * t1_2, 2 * t1, 1, 0,  //
      t2_3, t2_2, t2, 1,       //
      3 * t2_2, 2 * t2, 1, 0   //
      ;

  Eigen::Vector4d v;
  v << x1, dx1, x2, dx2;

  Eigen::Vector4d abcd = M.inverse() * v;

  struct CubicSpline::Polynom polynom = { abcd[0], abcd[1], abcd[2], abcd[3] };

  return polynom;
}

void CubicSpline::compute_splines()
{
  _splines.clear();
  if (_points.size() < 2)
  {
    return;
  }

  for (size_t i = 1; i < _points.size(); i++)
  {
    double t_start = _points[i - 1].t;
    struct Spline spline = { fit(_points[i - 1].t - t_start, _points[i - 1].x, _points[i - 1].dx,
                                 _points[i].t - t_start, _points[i].x, _points[i].dx),
                             _points[i - 1].t, _points[i].t };

    _splines.push_back(spline);
  }
}

}  // namespace placo::tools
