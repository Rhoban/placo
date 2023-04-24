#include "placo/trajectory/cubic_spline.h"
#include "placo/utils.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace placo
{
CubicSpline::CubicSpline(bool angular) : angular(angular)
{
}

void CubicSpline::add_point(double pos, double val, double delta)
{
  // Discontinuity of angle
  if (angular && _points.size() > 0)
  {
    val = _points.back().value + wrap_angle(val - _points.back().value);
  }
  struct Point point = { pos, val, delta };

  if (_points.size() > 0 && pos <= _points.back().position)
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
  return _splines[_splines.size() - 1].max - _splines[0].min;
}

/**
 * Return the spline interpolation
 * for given x position
 */
double CubicSpline::interpolation(double x, CubicSpline::ValueType type)
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
      return _points.front().value;
    }
    else
    {
      return _points.front().delta;
    }
  }
  else
  {
    if (x < _splines.front().min)
    {
      x = _splines.front().min;
    }
    if (x > _splines.back().max)
    {
      x = _splines.back().max;
    }

    for (size_t i = 0; i < _splines.size(); i++)
    {
      if (x >= _splines[i].min && x <= _splines[i].max)
      {
        if (type == Value)
        {
          return polynom_value(x, _splines[i].poly);
        }
        else if (type == Speed)
        {
          return polynom_diff(x, _splines[i].poly);
        }
      }
    }
    return 0.0;
  }
}

double CubicSpline::pos(double x)
{
  return interpolation(x, Value);
}

double CubicSpline::vel(double x)
{
  return interpolation(x, Speed);
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

CubicSpline::Polynom CubicSpline::fit(double t1, double val1, double delta1, double t2, double val2, double delta2)
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
  v << val1, delta1, val2, delta2;

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
    if (fabs(_points[i - 1].position - _points[i].position) < 0.00001)
    {
      continue;
    }
    struct Spline spline = { fit(_points[i - 1].position, _points[i - 1].value, _points[i - 1].delta,
                                 _points[i].position, _points[i].value, _points[i].delta),
                             _points[i - 1].position, _points[i].position };

    _splines.push_back(spline);
  }
}

}  // namespace placo
