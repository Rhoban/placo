#include <iostream>
#include "placo/tools/segment.h"

namespace placo::tools
{
Segment::Segment()
{
}

Segment::Segment(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
    : start(start), end(end)
{
}

bool Segment::is_parallel(const Segment& s, double epsilon)
{
    Eigen::Vector2d v1 = end - start;
    Eigen::Vector2d v2 = s.end - s.start;
    return abs(v1.x() * v2.y() - v1.y() * v2.x()) < epsilon;
}

bool Segment::is_point_aligned(const Eigen::Vector2d& point, double epsilon)
{
    if (abs((start - point).norm()) < epsilon)
    {
        return true;
    }
    double alpha_x = (point.x() - start.x()) / (end.x() - start.x());
    double alpha_y = (point.y() - start.y()) / (end.y() - start.y());
    return abs(alpha_x - alpha_y) < epsilon;
}

bool Segment::is_collinear(const Segment& s, double epsilon)
{
    return is_point_aligned(s.start, epsilon) && is_point_aligned(s.end, epsilon);
}

bool Segment::is_point_in_segment(const Eigen::Vector2d& point, double epsilon)
{
    Eigen::Vector2d v1 = end - start;
    Eigen::Vector2d v2 = point - start;
    return is_collinear(Segment(start, point), epsilon) && v1.dot(v2) >= 0 && v1.dot(v2) <= v1.dot(v1);
}

bool Segment::intersects(Segment& s)
{
    if (is_parallel(s))
    {
        throw std::runtime_error("Can't compute intersection of parallels");
    }
    return line_pass_through(s) && s.line_pass_through(*this);
}

bool Segment::line_pass_through(const Segment& s)
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
    double t = (v2.y() * (p2.x() - p1.x()) + v2.x() * (p1.y() - p2.y())) / det;
    return t >= 0 && t <= 1;
}

Eigen::Vector2d Segment::lines_intersection(const Segment& s)
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
    double t = (v2.y() * (p2.x() - p1.x()) + v2.x() * (p1.y() - p2.y())) / det;
    return p1 + t * v1;
}
}  // namespace placo::tools

