#pragma once

#include <Eigen/Dense>

namespace placo::tools
{
class Segment
{
public:
    Segment();
    Segment(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

    Eigen::Vector2d start;
    Eigen::Vector2d end;

    /**
     * @brief Checks if this segment is parallel to another one.
     * @param s The segment to check parallelism with.
     * @param epsilon To account for floating point errors.
     * @return True if the segments are parallel.
     */
    bool is_parallel(const Segment& s, double epsilon = 1e-6);

    /**
     * @brief Checks if a point is aligned with this segment.
     * @param point The point to check alignment with.
     * @param epsilon To account for floating point errors.
     * @return True if the point is aligned with the segment.
     */
    bool is_point_aligned(const Eigen::Vector2d& point, double epsilon = 1e-6);

    /**
     * @brief Checks if this segment is collinear with another one.
     * @param s The segment to check collinearity with.
     * @param epsilon To account for floating point errors.
     * @return True if the segments are collinear.
     */
    bool is_collinear(const Segment& s, double epsilon = 1e-6);

    /**
     * @brief Checks if a point is in the segment.
     * @param point The point to check.
     * @param epsilon To account for floating point errors.
     * @return True if the point is in the segment.
     */
    bool is_point_in_segment(const Eigen::Vector2d& point, double epsilon = 1e-6);

    /**
     * @brief Checks if there is an intersection between this segment and another one, i.e. if the 
     * intersection of their guiding lines is a point of both segments.
     * @param s The other segment.
     * @return True if there is an intersection.
     */
    bool intersects(Segment& s);

    /**
     * @brief Checks if the guiding line of another segment pass through this segment, i.e. if the 
     * intersection between the guiding lines of this segment and another one is a point of this segment.
     * @param s The other segment.
     * @return True if the intersection is a point of this segment.
     */
    bool line_pass_through(const Segment& s);

    /**
     * @brief Return the intersection between the guiding lines of this segment and another one.
     * @param s The other segment.
     * @return The intersection point.
     */
    Eigen::Vector2d lines_intersection(const Segment& s);

};
}  // namespace placo::tools
