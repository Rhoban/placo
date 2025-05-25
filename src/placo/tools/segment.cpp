#include <iostream>
#include "placo/tools/segment.h"

namespace placo::tools
{
Segment::Segment()
{
}

Segment::Segment(const Eigen::Vector2d& start, const Eigen::Vector2d& end) : start(start), end(end)
{
}

double Segment::norm()
{
  return (end - start).norm();
}

bool Segment::is_parallel(const Segment& s, double epsilon)
{
  Eigen::Vector2d v1 = end - start;
  Eigen::Vector2d v2 = s.end - s.start;
  return fabs(v1.x() * v2.y() - v1.y() * v2.x()) / (v1.norm() * v2.norm()) < epsilon;
}

bool Segment::is_point_aligned(const Eigen::Vector2d& point, double epsilon)
{
  if (fabs((start - point).norm()) < epsilon)
  {
    return true;
  }
  return is_parallel(Segment(start, point), epsilon);
}

bool Segment::is_segment_aligned(const Segment& s, double epsilon)
{
  return is_point_aligned(s.start, epsilon) && is_point_aligned(s.end, epsilon);
}

bool Segment::is_point_in_segment(const Eigen::Vector2d& point, double epsilon)
{
  Eigen::Vector2d v1 = end - start;
  Eigen::Vector2d v2 = point - start;
  return is_segment_aligned(Segment(start, point), epsilon) && v1.dot(v2) >= 0 && v1.dot(v2) <= v1.dot(v1);
}

bool Segment::intersects(Segment& s)
{
  if (is_parallel(s))
  {
    throw std::runtime_error("Can't compute intersection of parallels");
  }
  return line_pass_through(s) && s.line_pass_through(*this);
}

std::pair<double, double> Segment::get_lambdas(const Segment& s)
{
  if (is_parallel(s))
  {
    throw std::runtime_error("Can't compute intersection of parallels");
  }

  Eigen::Vector2d v1 = end - start;
  Eigen::Vector2d v2 = s.end - s.start;
  Eigen::Vector2d p1 = start;
  Eigen::Vector2d p2 = s.start;
  double det = v1.x() * v2.y() - v1.y() * v2.x();
  double l1 = (v2.y() * (p2.x() - p1.x()) + v2.x() * (p1.y() - p2.y())) / det;
  double l2 = (v1.y() * (p2.x() - p1.x()) + v1.x() * (p1.y() - p2.y())) / det;
  return std::make_pair(l1, l2);
}

bool Segment::line_pass_through(const Segment& s)
{
  auto lambdas = get_lambdas(s);
  return lambdas.first >= 0 && lambdas.first <= 1;
}

bool Segment::half_line_pass_through(const Segment& s)
{
  auto lambdas = get_lambdas(s);
  return lambdas.first >= 0 && lambdas.first <= 1 && lambdas.second >= 0;
}

Eigen::Vector2d Segment::lines_intersection(const Segment& s)
{
  auto lambdas = get_lambdas(s);
  Eigen::Vector2d v = end - start;
  return start + lambdas.first * v;
}
}  // namespace placo::tools
