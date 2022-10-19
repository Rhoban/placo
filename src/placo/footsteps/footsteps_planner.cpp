/** \file */

#include "rhoban_utils/history/history.h"
#include "placo/footsteps/footsteps_planner.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>


BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
typedef boost::tuple<double, double> b_point;
typedef boost::geometry::model::polygon<b_point> b_polygon;

/**
 * TODO: We always take at least two steps here even if the target is initially where we are
 * TODO: The accessibility could be refined instead of relying on one hypercube
 * TODO: How can we make sure that legs doesn't collide aech other ?
 * TODO: Feet dimensions should come from the model
 * TODO: The foot that moves first should be defineable
 */

// Dimension of the accessibility window for the opposite foot
const double accessibility_width = 0.05;
const double accessibility_length = 0.1;
const double accessibility_yaw = 0.2;

// Distance where the robot walks forward instead of aligning with target
const double place_threshold = 0.5;

// Foot dimension
const double foot_width = 0.1;
const double foot_length = 0.15;


namespace placo
{
/**
 * @brief Creation of the support polygon of a footstep
 * 
 * @return New polygon contained in Footstep
 */
const std::vector<Eigen::Vector2d>& FootstepsPlanner::Footstep::support_polygon()
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
      Eigen::Vector3d corner = frame * Eigen::Vector3d(sxsy.first * foot_length / 2, sxsy.second * foot_width / 2, 0);
      Eigen::Vector2d point(corner.x(), corner.y());
      polygon.push_back(point);
    }
    computed_polygon = true;
  }

  return polygon;
}

/**
 * @brief Generation of the support area of the foot(s) depending on whether it is a single or double support
 * 
 * @return New polygon contained in Support
 */
const std::vector<Eigen::Vector2d>& FootstepsPlanner::Support::support_polygon()
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

/**
* TODO: Documentation
*/ 
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
      f = rhoban_utils::averageFrames(f, footstep.frame, 1. / n);
    }

    n += 1;
  }

  return f;
}

/**
* TODO: Documentation
*/ 
Eigen::Affine3d FootstepsPlanner::Support::frame(Side side)
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

/**
 * Constructor
 * @param model_ model
 */
FootstepsPlanner::FootstepsPlanner(Eigen::Affine3d T_world_right, double foots_spacing) : foots_spacing(foots_spacing)
{
  configuration_foots.initial_right_foot = T_world_right;
  configuration_foots.initial_left_foot = T_world_right;
  configuration_foots.initial_left_foot.translation().y() += foots_spacing;
}

void FootstepsPlanner::set_initial(Eigen::Vector3d initial){
  Eigen::Affine3d temp;
  temp.translation() = Eigen::Vector3d(initial.x(),initial.y(),0);
  temp.linear() = Eigen::AngleAxisd(initial.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

  configuration_foots.initial_right_foot = temp;

  temp.translation().y() += foots_spacing;
  configuration_foots.initial_left_foot = temp;
}

void FootstepsPlanner::set_support(FootstepsPlanner::Side side) {
  side_support = side;
}

void FootstepsPlanner::set_initial(Eigen::Vector3d initial_right, Eigen::Vector3d initial_left){
  Eigen::Affine3d temp;
  temp.translation() = Eigen::Vector3d(initial_right.x(), initial_right.y(), 0);
  temp.linear() = Eigen::AngleAxisd(initial_right.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  configuration_foots.initial_right_foot = temp;

  temp.translation() = Eigen::Vector3d(initial_left.x(), initial_left.y(), 0);
  temp.linear() = Eigen::AngleAxisd(initial_left.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

  if(initial_left.y() == -1){
    temp.translation().y() = initial_right.y() + foots_spacing;
  }
  configuration_foots.initial_left_foot = temp;
}

/**
 * @brief Plans footsteps to bring the robot to the target position and orientation
 * 
 * @param target_left Position to be reached by the left foot
 * @param target_right Position to be reached by the right foot
 * @param debug Activation or not of the debug mode
 * @return List of footsteps to reach the targets
 */
std::vector<FootstepsPlanner::Footstep> FootstepsPlanner::plan(Eigen::Affine3d target_left,
                                                               Eigen::Affine3d target_right, bool debug)
{
  std::vector<FootstepsPlanner::Footstep> footsteps;

  Eigen::Affine3d target = rhoban_utils::averageFrames(target_left, target_right, 0.5);

  auto current_left = configuration_foots.initial_left_foot;

  auto current_right = configuration_foots.initial_right_foot;

  // if (debug)
  // {
  //   py_print_step("initial", current_left);
  //   py_print_step("initial", current_right);
  //   py_print_step("target", target_left);
  //   py_print_step("target", target_right);
  // }

  auto support_side = side_support;

  bool left_arrived = false;
  bool right_arrived = false;
  int steps = 0;

  while (!left_arrived || !right_arrived)
  {
    steps += 1;

    bool arrived = true;

    Eigen::Affine3d support_to_world = (support_side == Left) ? current_left : current_right;
    // Floating foot to current frame
    Eigen::Affine3d other_center_to_support = Eigen::Affine3d::Identity();
    Eigen::Affine3d center_to_support = Eigen::Affine3d::Identity();
    // Floating foot target to current frame
    Eigen::Affine3d target_to_support =
        support_to_world.inverse() * ((support_side == Left) ? target_right : target_left);

    if (support_side == Left)
    {
      other_center_to_support.translation().y() = -foots_spacing;
      center_to_support.translation().y() = -foots_spacing / 2.;
    }
    else
    {
      other_center_to_support.translation().y() = foots_spacing;
      center_to_support.translation().y() = foots_spacing / 2.;
    }

    // Updating the position
    Eigen::Vector3d error = target_to_support.translation() - other_center_to_support.translation();

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
          (support_to_world.inverse() * target).translation() - center_to_support.translation();
      error_yaw = atan2(target_to_center.y(), target_to_center.x());
    }
    else
    {
      error_yaw = model.frame_yaw(target_to_support.rotation());
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
    new_step.translation() = other_center_to_support.translation() + error;
    new_step.linear() = Eigen::AngleAxisd(error_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Going to next step
    FootstepsPlanner::Footstep footstep;
    footstep.side = (support_side == model.Left) ? model.Right : model.Left;
    footstep.frame = support_to_world * new_step;
    footsteps.push_back(footstep);

    if (support_side == model.Left)
    {
      right_arrived = arrived;
      current_right = footstep.frame;
      support_side = model.Right;
    }
    else
    {
      left_arrived = arrived;
      current_left = footstep.frame;
      support_side = model.Left;
    }

    if (debug)
    {
      py_print_polygon("step", footstep.support_polygon());
    }
  }

  return footsteps;
}

/**
 * @brief Plans footsteps to bring the robot to the target position and orientation
 * 
 * @param target_left Position to be reached by robot
 * @param debug Activation or not of the debug mode
 * @return List of footsteps to reach the targets
 */
std::vector<FootstepsPlanner::Footstep> FootstepsPlanner::plan(Eigen::Affine3d target, bool debug)
{
  Eigen::Affine3d target_left = target;
  target_left.translation() = target * Eigen::Vector3d(0, foots_spacing / 2., 0);
  Eigen::Affine3d target_right = target;
  target_right.translation() = target * Eigen::Vector3d(0, -foots_spacing / 2., 0);

  return plan(target_left, target_right, debug);
}

/**
 * @brief Plans footsteps to bring the robot to the target position and orientation
 * 
 * @param side Side of the robot to be moved to the target
 * @param frame Frame of the target to be reached
 * @param debug Activation or not of the debug mode
 * @return List of footsteps to reach the targets
 */
std::vector<FootstepsPlanner::Footstep> FootstepsPlanner::plan(FootstepsPlanner::Side side, Eigen::Affine3d frame,
                                                               bool debug)
{
  if (side == Left)
  {
    Eigen::Affine3d right_frame = frame;
    right_frame.translation() += frame.rotation() * Eigen::Vector3d(0., -foots_spacing, 0.);
    return plan(frame, right_frame, debug);
  }
  else
  {
    Eigen::Affine3d left_frame = frame;
    left_frame.translation() += frame.rotation() * Eigen::Vector3d(0., foots_spacing, 0.);
    return plan(left_frame, frame, debug);
  }
}


/**
 * @brief Make sequential double / single support phases from a footsteps plan
 * 
 * @param footsteps List of footsteps to reach the target
 * @return Lists of footsteps alternating between single and double support phases
 */
std::vector<FootstepsPlanner::Support> FootstepsPlanner::make_supports(const std::vector<Footstep>& footsteps)
{
  std::vector<FootstepsPlanner::Support> supports;

  // Building current (double) support
  Footstep left_footstep;
  Eigen::Affine3d left_initial;
  left_initial.translation() = initial_pos.translation();
  left_initial.translation().y() += foots_spacing;
  left_initial.linear() = initial_pos.linear();

  left_footstep.frame = configuration_foots.initial_left_foot;
  left_footstep.side = Side::Left;

  Footstep right_footstep;
  right_footstep.frame  = configuration_foots.initial_right_foot;
  right_footstep.side = Side::Right;

  Support current_support;
  current_support.footsteps.push_back(left_footstep);
  current_support.footsteps.push_back(right_footstep);
  supports.push_back(current_support);

  // Previous footstep
  Footstep last_footstep = left_footstep;
  if(side_support == Side::Right){
    last_footstep = right_footstep;
  }

  for (auto& footstep : footsteps)
  {
    // The last footstep is now a support, the other leg is flying
    Support single_phase;
    single_phase.footsteps.push_back(last_footstep);
    supports.push_back(single_phase);

    // The new foostep is on the ground, a double support phase is occurring
    Support double_phase;
    double_phase.footsteps.push_back(last_footstep);
    double_phase.footsteps.push_back(footstep);
    supports.push_back(double_phase);

    last_footstep = footstep;
  }

  return supports;
}
}