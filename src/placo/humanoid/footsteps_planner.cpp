#include "placo/humanoid/footsteps_planner.h"
#include "placo/tools/utils.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/polygon.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<double, double> b_point;
typedef boost::geometry::model::polygon<b_point> b_polygon;

namespace placo::humanoid
{

FootstepsPlanner::Footstep::Footstep(double foot_width, double foot_length)
  : foot_width(foot_width), foot_length(foot_length)
{
}

std::vector<Eigen::Vector2d> FootstepsPlanner::Footstep::compute_polygon(double margin)
{
  std::vector<Eigen::Vector2d> polygon;

  // Making a clockwise polygon
  std::vector<std::pair<double, double>> contour = {
    std::make_pair(-1., 1.),
    std::make_pair(1., 1.),
    std::make_pair(1., -1.),
    std::make_pair(-1., -1.),
  };

  for (auto sxsy : contour)
  {
    Eigen::Vector3d corner =
        frame * Eigen::Vector3d(sxsy.first * (margin + foot_length / 2), sxsy.second * (margin + foot_width / 2), 0.);
    Eigen::Vector2d point(corner.x(), corner.y());
    polygon.push_back(point);
  }

  return polygon;
}

std::vector<Eigen::Vector2d> FootstepsPlanner::Footstep::support_polygon()
{
  if (!computed_polygon)
  {
    polygon = compute_polygon();
    computed_polygon = true;
  }

  return polygon;
}

bool FootstepsPlanner::Footstep::polygon_contains(std::vector<Eigen::Vector2d>& polygon, Eigen::Vector2d point)
{
  Eigen::Vector2d last_point = polygon[polygon.size() - 1];

  for (auto current_point : polygon)
  {
    Eigen::Vector2d v = current_point - last_point;
    Eigen::Vector2d n(v.y(), -v.x());

    if (n.dot(point - last_point) < 0)
    {
      return false;
    }

    last_point = current_point;
  }

  return true;
}

bool FootstepsPlanner::Footstep::overlap(Footstep& other, double margin)
{
  std::vector<Eigen::Vector2d> support1 = compute_polygon(margin);
  std::vector<Eigen::Vector2d> support2 = other.compute_polygon(margin);

  for (auto pt : support1)
  {
    if (polygon_contains(support2, pt))
    {
      return true;
    }
  }
  for (auto pt : support2)
  {
    if (polygon_contains(support1, pt))
    {
      return true;
    }
  }

  return false;
}

bool FootstepsPlanner::Footstep::operator==(const Footstep& other)
{
  return side == other.side && frame.isApprox(other.frame);
}

FootstepsPlanner::Support::Support()
{
}

FootstepsPlanner::Support::Support(std::vector<Footstep> footsteps) : footsteps(footsteps)
{
}

std::vector<Eigen::Vector2d> FootstepsPlanner::Support::support_polygon()
{
  if (!computed_polygon)
  {
    polygon.clear();

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
      f = tools::interpolate_frames(f, footstep.frame, 1. / n);
    }

    n += 1;
  }
  return f;
}

Eigen::Affine3d FootstepsPlanner::Support::footstep_frame(HumanoidRobot::Side side)
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

void FootstepsPlanner::Support::apply_offset(Eigen::Vector2d offset)
{
  for (auto& footstep : footsteps)
  {
    footstep.frame.translation().x() += offset.x();
    footstep.frame.translation().y() += offset.y();
    footstep.computed_polygon = false;
  }
  computed_polygon = false;
}

HumanoidRobot::Side FootstepsPlanner::Support::side()
{
  if (footsteps.size() > 1)
  {
    throw std::runtime_error("Asking side() for a double support (call is_both() before)");
  }

  return footsteps[0].side;
}

bool FootstepsPlanner::Support::is_both()
{
  return footsteps.size() == 2;
}

FootstepsPlanner::Support operator*(Eigen::Affine3d T, const FootstepsPlanner::Support& support)
{
  FootstepsPlanner::Support new_support = support;

  for (auto& footstep : new_support.footsteps)
  {
    footstep.frame = T * footstep.frame;
    footstep.computed_polygon = false;
  }
  new_support.computed_polygon = false;

  return new_support;
}

bool FootstepsPlanner::Support::operator==(const Support& other)
{
  if (footsteps.size() != other.footsteps.size())
  {
    return false;
  }
  for (int k = 0; k < (int)footsteps.size(); k++)
  {
    if (!(footsteps[k] == other.footsteps[k]))
    {
      return false;
    }
  }

  return true;
}

FootstepsPlanner::FootstepsPlanner(HumanoidParameters& parameters) : parameters(parameters)
{
}

std::vector<FootstepsPlanner::Support> FootstepsPlanner::make_supports(
    std::vector<FootstepsPlanner::Footstep> footsteps, double t_start, bool start, bool middle, bool end)
{
  std::vector<Support> supports;

  if (footsteps.size() > 2)
  {
    if (start)
    {
      // Creating the first (double-support) initial state
      Support support({ footsteps[0], footsteps[1] });
      support.start = true;
      support.t_start = t_start;
      supports.push_back(support);
    }
    else
    {
      Support support({ footsteps[0] });
      support.t_start = t_start;
      supports.push_back(support);

      if (middle)
      {
        Support double_support({ footsteps[0], footsteps[1] });
        double_support.t_start = t_start;
        supports.push_back(double_support);
      }
    }

    // Adding single/double support phases
    for (int step = 1; step < (int)footsteps.size() - 1; step++)
    {
      Support single_support({ footsteps[step] });
      supports.push_back(single_support);

      bool is_end = (step == (int)footsteps.size() - 2);

      if ((!is_end && middle))
      {
        Support double_support({ footsteps[step], footsteps[step + 1] });
        supports.push_back(double_support);
      }
    }
  }

  if (end)
  {
    // Creating the first (double-support) initial state
    Support support({ footsteps[footsteps.size() - 2], footsteps[footsteps.size() - 1] });
    support.end = true;
    supports.push_back(support);
  }

  return supports;
}

FootstepsPlanner::Footstep FootstepsPlanner::create_footstep(HumanoidRobot::Side side, Eigen::Affine3d T_world_foot)
{
  FootstepsPlanner::Footstep footstep(parameters.foot_width, parameters.foot_length);

  footstep.side = side;
  footstep.frame = T_world_foot;

  return footstep;
}

FootstepsPlanner::Footstep FootstepsPlanner::opposite_footstep(FootstepsPlanner::Footstep footstep, double d_x,
                                                               double d_y, double d_theta)
{
  footstep.frame = parameters.opposite_frame(footstep.side, footstep.frame, d_x, d_y, d_theta);
  footstep.side = HumanoidRobot::other_side(footstep.side);

  return footstep;
}

FootstepsPlanner::Footstep FootstepsPlanner::clipped_opposite_footstep(Footstep footstep, double d_x, double d_y,
                                                                       double d_theta)
{
  Eigen::Vector3d step(d_x, d_y, d_theta);

  if (footstep.side == HumanoidRobot::Side::Left)
  {
    step.y() -= parameters.walk_dtheta_spacing * fabs(step.z());
  }
  else
  {
    step.y() += parameters.walk_dtheta_spacing * fabs(step.z());
  }

  step = parameters.ellipsoid_clip(step);

  for (int k = 0; k < 32; k++)
  {
    Footstep new_footstep = opposite_footstep(footstep, step.x(), step.y(), step.z());

    if (new_footstep.overlap(footstep, 1e-2))
    {
      step *= 0.9;
    }
    else
    {
      return new_footstep;
    }
  }

  return opposite_footstep(footstep, step.x(), step.y(), step.z());
}

std::vector<FootstepsPlanner::Footstep> FootstepsPlanner::plan(HumanoidRobot::Side flying_side,
                                                               Eigen::Affine3d T_world_left,
                                                               Eigen::Affine3d T_world_right)
{
  std::vector<Footstep> footsteps;

  // Including initial footsteps
  auto current_side = flying_side;
  auto T_world_current_frame = (current_side == HumanoidRobot::Side::Left) ? T_world_left : T_world_right;
  footsteps.push_back(create_footstep(current_side, T_world_current_frame));

  current_side = HumanoidRobot::other_side(current_side);
  T_world_current_frame = (current_side == HumanoidRobot::Side::Left) ? T_world_left : T_world_right;
  footsteps.push_back(create_footstep(current_side, T_world_current_frame));

  // Calling specific implementation
  plan_impl(footsteps, flying_side, T_world_left, T_world_right);

  return footsteps;
}
}  // namespace placo::humanoid