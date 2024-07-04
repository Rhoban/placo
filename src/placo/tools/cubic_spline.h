#pragma once

#include <vector>
#include <algorithm>

namespace placo::tools
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
   * @brief Spline duration
   * @return duration in seconds
   */
  double duration() const;

  /**
   * @brief Adds a point in the spline
   * @param t time
   * @param x value
   * @param dx speed
   */
  void add_point(double t, double x, double dx);

  /**
   * @brief Clears the spline
   */
  void clear();

  /**
   * @brief Retrieve the position at a given time
   * @param t time
   * @return position
   */
  double pos(double t);

  /**
   * @brief Retrieve velocity at a given time
   * @param t time
   * @return velocity
   */
  double vel(double x);

  /**
   * @brief Retrieve acceleration at a given time
   * @param t time
   * @return acceleration
   */
  double acc(double x);

  enum ValueType
  {
    Value,
    Speed,
    Acceleration
  };
  double interpolation(double x, ValueType type);

  /**
   * @brief Access internal points container
   * @return points
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
   * Polynom diff. value
   */
  static double polynom_diff2(double t, const Polynom& p);

  /**
   * Fit a polynom
   */
  static Polynom fit(double t1, double x1, double dx1, double t2, double x2, double dx2);

  /**
   * Recompute splines interpolation model
   */
  void compute_splines();
};

}  // namespace placo::tools
