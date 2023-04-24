#pragma once

#include <vector>
#include <algorithm>

namespace placo
{
class CubicSpline
{
public:
  CubicSpline(bool angular = false);

  struct Point
  {
    double position;
    double value;
    double delta;
  };

  typedef std::vector<Point> Points;

  /**
   * Spline duration
   */
  double duration() const;

  /**
   * Add a point with its x position, y value and
   * its derivative slope
   */
  void add_point(double pos, double val, double delta);

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
    double min;
    double max;
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
   * Fit a polynom between 0 and 1 with
   * given value and slope
   */
  static Polynom fit(double t1, double val1, double delta1, double t2, double val2, double delta2);

  /**
   * Recompute splines interpolation model
   */
  void compute_splines();
};

}  // namespace placo
