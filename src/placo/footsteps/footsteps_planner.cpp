#include "placo/footsteps/footsteps_planner.h"
#include "placo/utils.h"
#include "rhoban_utils/history/history.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/polygon.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<double, double> b_point;
typedef boost::geometry::model::polygon<b_point> b_polygon;

namespace placo
{
FootstepsPlanner::Footstep::Footstep(double foot_width, double foot_length)
  : foot_width(foot_width), foot_length(foot_length)
{
}

std::vector<Eigen::Vector2d> FootstepsPlanner::Footstep::support_polygon()
{
  if (!computed_polygon)
  {
    // Making a clockwise polygon
    std::vector<std::pair<double, double>> contour = {
      std::make_pair(-1., 1.),
      std::make_pair(1., 1.),
      std::make_pair(1., -1.),
      std::make_pair(-1., -1.),
    };

    for (auto sxsy : contour)
    {
      Eigen::Vector3d corner = frame * Eigen::Vector3d(sxsy.first * foot_length / 2, sxsy.second * foot_width / 2, 0.);
      Eigen::Vector2d point(corner.x(), corner.y());
      polygon.push_back(point);
    }
    computed_polygon = true;
  }

  return polygon;
}

std::vector<Eigen::Vector2d> FootstepsPlanner::Support::support_polygon()
{
  if (!computed_polygon)
  {
    b_polygon poly, hull;
    for (auto& footstep : footsteps)
    {
      for (auto& pt : footstep.support_polygon())
      {
        boost::geometry::append(poly, b_point(pt.x(), pt.y()));
      }
    }

    // Boost convex hull is also clockwise
    boost::geometry::convex_hull(poly, hull);

    for (auto& pt : hull.outer())
    {
      polygon.push_back(Eigen::Vector2d(pt.get<0>(), pt.get<1>()));
    }
    polygon.pop_back();
    computed_polygon = true;
  }

  return polygon;
}

Eigen::Affine3d FootstepsPlanner::Support::frame()
{
  Eigen::Affine3d f;
  int n = 1;

  for (auto& footstep : footsteps)
  {
    if (n == 1)
    {
      f = footstep.frame;
    }
    else
    {
      f = interpolate_frames(f, footstep.frame, 1. / n);
    }

    n += 1;
  }

  return f;
}

Eigen::Affine3d FootstepsPlanner::Support::frame(HumanoidRobot::Side side)
{
  for (auto& footstep : footsteps)
  {
    if (footstep.side == side)
    {
      return footstep.frame;
    }
  }

  throw std::logic_error("Asked for a frame that doesn't exist");
}

FootstepsPlanner::FootstepsPlanner(HumanoidRobot::Side initial_side, Eigen::Affine3d T_world_left,
                                   Eigen::Affine3d T_world_right)
  : initial_side(initial_side), T_world_left(T_world_left), T_world_right(T_world_right)
{
}

FootstepsPlanner::FootstepsPlanner(std::string initial_side, Eigen::Affine3d T_world_left,
                                   Eigen::Affine3d T_world_right)
  : initial_side(HumanoidRobot::string_to_side(initial_side)), T_world_left(T_world_left), T_world_right(T_world_right)
{
}

bool FootstepsPlanner::Footstep::operator==(const Footstep& other)
{
  return side == other.side && frame.isApprox(other.frame);
}

bool FootstepsPlanner::Support::operator==(const Support& other)
{
  if (footsteps.size() != other.footsteps.size())
  {
    return false;
  }
  for (int k = 0; k < footsteps.size(); k++)
  {
    if (!(footsteps[k] == other.footsteps[k]))
    {
      return false;
    }
  }

  return true;
}

HumanoidRobot::Side FootstepsPlanner::Support::side()
{
  if (footsteps.size() == 2)
  {
    return HumanoidRobot::Side::Both;
  }
  else
  {
    return footsteps[0].side;
  }
}

void FootstepsPlanner::make_supports(bool start, bool middle, bool end)
{
  std::vector<Support> computed_supports;

  if (start)
  {
    // Creating the first (double-support) initial state
    Support support;
    support.start_end = true;
    support.footsteps = { footsteps[0], footsteps[1] };
    computed_supports.push_back(support);
  }
  else
  {
    Support support;
    support.start_end = true;
    support.footsteps = { footsteps[0] };
    computed_supports.push_back(support);
  }

  // Adding single/double support phases
  for (int step = 1; step < footsteps.size() - 1; step++)
  {
    Support single_support;
    single_support.footsteps = { footsteps[step] };
    computed_supports.push_back(single_support);

    bool is_end = (step == footsteps.size() - 2);

    if ((!is_end && middle) || (is_end && end))
    {
      Support double_support;
      double_support.footsteps = { footsteps[step], footsteps[step + 1] };

      if (is_end)
      {
        double_support.start_end = true;
      }

      computed_supports.push_back(double_support);
    }
  }

  supports = computed_supports;
}

void FootstepsPlanner::replan(HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_left,
                              Eigen::Affine3d T_world_right)
{
  initial_side = flying_side;

  T_world_left = T_world_left;
  T_world_right = T_world_right;

  plan();

  bool double_support = parameters.double_support_duration / parameters.dt > 1;
  make_supports(double_support, double_support, true);
  new_supports = true;
}
}  // namespace placo