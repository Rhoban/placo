#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/utils.h"
#include "rhoban_utils/history/history.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/polygon.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<double, double> b_point;
typedef boost::geometry::model::polygon<b_point> b_polygon;

/**
 * TODO: We always take at least two steps here even if the target is initially
 * where we are
 * TODO: The accessibility could be refined instead of relying on one hypercube
 * TODO: How can we make sure that legs doesn't collide aech other ?
 * TODO: Feet dimensions should come from the model
 */

namespace placo
{
FootstepsPlannerNaive::FootstepsPlannerNaive(Side initial_side, Eigen::Affine3d T_world_left,
                                             Eigen::Affine3d T_world_right, double feet_spacing)
  : FootstepsPlanner(initial_side, T_world_left, T_world_right, feet_spacing)
{
}

FootstepsPlannerNaive::FootstepsPlannerNaive(std::string initial_side, Eigen::Affine3d T_world_left,
                                             Eigen::Affine3d T_world_right, double feet_spacing)
  : FootstepsPlanner(initial_side, T_world_left, T_world_right, feet_spacing)
{
}

std::vector<FootstepsPlanner::Footstep> FootstepsPlannerNaive::plan(Eigen::Affine3d T_world_targetLeft,
                                                                    Eigen::Affine3d T_world_targetRight)
{
  std::vector<FootstepsPlanner::Footstep> footsteps;

  Eigen::Affine3d T_world_target = rhoban_utils::averageFrames(T_world_targetLeft, T_world_targetRight, 0.5);

  auto T_world_currentLeft = T_world_left;
  auto T_world_currentRight = T_world_right;
  auto support_side = initial_side;

  bool left_arrived = false;
  bool right_arrived = false;
  int steps = 0;

  // Including initial footsteps, which are current frames
  FootstepsPlanner::Footstep footstep(foot_width, foot_length);
  footstep.side = support_side == Side::Left ? Side::Right : Side::Left;
  footstep.frame = support_side == Side::Left ? T_world_right : T_world_left;
  footsteps.push_back(footstep);

  footstep.side = support_side;
  footstep.frame = support_side == Side::Left ? T_world_left : T_world_right;
  footsteps.push_back(footstep);

  while ((!left_arrived || !right_arrived) && steps < max_steps)
  {
    steps += 1;

    bool arrived = true;

    // The current support in the world
    Eigen::Affine3d T_world_support = (support_side == Left) ? T_world_currentLeft : T_world_currentRight;

    // Floating foot to current frame
    Eigen::Affine3d T_support_floatingIdle = Eigen::Affine3d::Identity();
    Eigen::Affine3d T_support_center = Eigen::Affine3d::Identity();

    // Expressing the target (for current flying foot) in the support foot
    Eigen::Affine3d T_support_target =
        T_world_support.inverse() * ((support_side == Left) ? T_world_targetRight : T_world_targetLeft);

    if (support_side == Left)
    {
      T_support_floatingIdle.translation().y() = -feet_spacing;
      T_support_center.translation().y() = -feet_spacing / 2.;
    }
    else
    {
      T_support_floatingIdle.translation().y() = feet_spacing;
      T_support_center.translation().y() = feet_spacing / 2.;
    }

    // Updating the position
    Eigen::Vector3d error = T_support_target.translation() - T_support_floatingIdle.translation();

    double rescale = 1.;

    if (error.x() < -accessibility_length)
    {
      rescale = std::min(rescale, -accessibility_length / error.x());
      arrived = false;
    }
    if (error.x() > accessibility_length)
    {
      rescale = std::min(rescale, accessibility_length / error.x());
      arrived = false;
    }
    if (error.y() < -accessibility_width)
    {
      rescale = std::min(rescale, -accessibility_width / error.y());
      arrived = false;
    }
    if (error.y() > accessibility_width)
    {
      rescale = std::min(rescale, accessibility_width / error.y());
      arrived = false;
    }

    double dist = error.norm();
    error = error * rescale;

    // Updating the yaw
    double error_yaw;

    if (dist > place_threshold)
    {
      Eigen::Vector3d target_to_center =
          (T_world_support.inverse() * T_world_target).translation() - T_support_center.translation();
      error_yaw = atan2(target_to_center.y(), target_to_center.x());
    }
    else
    {
      error_yaw = placo::frame_yaw(T_support_target.rotation());
    }

    if (error_yaw < -accessibility_yaw)
    {
      arrived = false;
      error_yaw = -accessibility_yaw;
    }
    if (error_yaw > accessibility_yaw)
    {
      arrived = false;
      error_yaw = accessibility_yaw;
    }

    // Computing new frame
    Eigen::Affine3d new_step;
    new_step.translation() = T_support_floatingIdle.translation() + error;
    new_step.linear() = Eigen::AngleAxisd(error_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Going to next step
    FootstepsPlanner::Footstep footstep(foot_width, foot_length);
    footstep.side = (support_side == Side::Left) ? Side::Right : Side::Left;
    footstep.frame = T_world_support * new_step;
    footsteps.push_back(footstep);

    if (support_side == Side::Left)
    {
      right_arrived = arrived;
      T_world_currentRight = footstep.frame;
      support_side = Side::Right;
    }
    else
    {
      left_arrived = arrived;
      T_world_currentLeft = footstep.frame;
      support_side = Side::Left;
    }
  }

  return footsteps;
}

}  // namespace placo