#pragma once

#include <vector>
#include <algorithm>

namespace placo::trajectory
{
class CubicSpline
{
public:
  CubicSpline(bool angular = false);

  struct Point
  {
    double t;
    double x;
    double dx;
  };

  typedef std::vector<Point> Points;

  /**
   * Spline duration
   */
  double duration() const;

  /**
   * Add a point with its time, x and dx
   */
  void add_point(double t, double x, double dx);

  void clear();

  /**
   * Return the spline interpolation
   * for given x position
   */
  double pos(double x);

  /**
   * Returns the spline speed interpolation
   * for given x position
   */
  double vel(double x);

  enum ValueType
  {
    Value,
    Speed
  };
  double interpolation(double x, ValueType type);

  /**
   * Access to internal Points container
   */
  const Points& points() const;

private:
  bool angular = false;
  bool dirty = true;

  struct Polynom
  {
    double a;
    double b;
    double c;
    double d;
  };

  struct Spline
  {
    Polynom poly;
    double t_start;
    double t_end;
  };

  typedef std::vector<Spline> Splines;

  /**
   * Spline Points container
   */
  Points _points;

  /**
   * Splines container
   */
  Splines _splines;

  /**
   * Fast exponentation to compute
   * given polynom value
   */
  static double polynom_value(double t, const Polynom& p);

  /**
   * Polynom diff. value
   */
  static double polynom_diff(double t, const Polynom& p);

  /**
   * Fit a polynom
   */
  static Polynom fit(double t1, double x1, double dx1, double t2, double x2, double dx2);

  /**
   * Recompute splines interpolation model
   */
  void compute_splines();
};

}  // namespace placo::trajectory
