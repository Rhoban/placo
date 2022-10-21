/** \file */

#include "placo/footsteps/footsteps_planner.h"
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
 * TODO: The foot that moves first should be defineable
 */

namespace placo {
FootstepsPlanner::Footstep::Footstep(double foot_width, double foot_length)
    : foot_width(foot_width), foot_length(foot_length) {}

std::vector<Eigen::Vector2d> FootstepsPlanner::Footstep::support_polygon() {
  if (!computed_polygon) {
    // Making a clockwise polygon
    std::vector<std::pair<double, double>> contour = {
        std::make_pair(-1., 1.),
        std::make_pair(1., 1.),
        std::make_pair(1., -1.),
        std::make_pair(-1., -1.),
    };

    for (auto sxsy : contour) {
      Eigen::Vector3d corner =
          frame * Eigen::Vector3d(sxsy.first * foot_length / 2,
                                  sxsy.second * foot_width / 2, 0);
      Eigen::Vector2d point(corner.x(), corner.y());
      polygon.push_back(point);
    }
    computed_polygon = true;
  }

  return polygon;
}

/**
 * @brief Generation of the support area of the foot(s) depending on whether it
 * is a single or double support
 *
 * @return New polygon contained in Support
 */
std::vector<Eigen::Vector2d> FootstepsPlanner::Support::support_polygon() {
  if (!computed_polygon) {
    b_polygon poly, hull;
    for (auto &footstep : footsteps) {
      for (auto &pt : footstep.support_polygon()) {
        boost::geometry::append(poly, b_point(pt.x(), pt.y()));
      }
    }

    // Boost convex hull is also clockwise
    boost::geometry::convex_hull(poly, hull);

    for (auto &pt : hull.outer()) {
      polygon.push_back(Eigen::Vector2d(pt.get<0>(), pt.get<1>()));
    }
    polygon.pop_back();
    computed_polygon = true;
  }

  return polygon;
}

/**
 * @brief Computes the frame for the support
 * @return The frame is the interpolation of all supports contained in the
 * structure
 */
Eigen::Affine3d FootstepsPlanner::Support::frame() {
  Eigen::Affine3d f;
  int n = 1;

  for (auto &footstep : footsteps) {
    if (n == 1) {
      f = footstep.frame;
    } else {
      f = placo::interpolate_frames(f, footstep.frame, 1. / n);
    }

    n += 1;
  }

  return f;
}

/**
 * @brief Returns the frame for a given side
 * @param side
 * @return The frame for this side
 */
Eigen::Affine3d FootstepsPlanner::Support::frame(Side side) {
  for (auto &footstep : footsteps) {
    if (footstep.side == side) {
      return footstep.frame;
    }
  }

  throw std::logic_error("Asked for a frame that doesn't exist");
}

FootstepsPlanner::FootstepsPlanner(Side initial_side,
                                   Eigen::Affine3d T_world_left,
                                   Eigen::Affine3d T_world_right,
                                   double feet_spacing)
    : initial_side(initial_side), T_world_left(T_world_left),
      T_world_right(T_world_right), feet_spacing(feet_spacing) {}

FootstepsPlanner::FootstepsPlanner(std::string initial_side,
                                   Eigen::Affine3d T_world_left,
                                   Eigen::Affine3d T_world_right,
                                   double feet_spacing)
    : initial_side(initial_side == "left" ? Left : Right),
      T_world_left(T_world_left), T_world_right(T_world_right),
      feet_spacing(feet_spacing) {}

bool FootstepsPlanner::Footstep::operator==(const Footstep &other) {
  return side == other.side && frame.isApprox(other.frame);
}

/**
 * @brief Plans footsteps to bring the robot to the target position and
 * orientation
 *
 * @param target_left Position to be reached by the left foot
 * @param target_right Position to be reached by the right foot
 * @param debug Activation or not of the debug mode
 * @return List of footsteps to reach the targets
 */
std::vector<FootstepsPlanner::Footstep>
FootstepsPlanner::plan(Eigen::Affine3d T_world_targetLeft,
                       Eigen::Affine3d T_world_targetRight) {

  std::vector<FootstepsPlanner::Footstep> footsteps;

  Eigen::Affine3d T_world_target =
      rhoban_utils::averageFrames(T_world_targetLeft, T_world_targetRight, 0.5);

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

  while ((!left_arrived || !right_arrived) && steps < max_steps) {
    steps += 1;

    bool arrived = true;

    // The current support in the world
    Eigen::Affine3d T_world_support =
        (support_side == Left) ? T_world_currentLeft : T_world_currentRight;

    // Floating foot to current frame
    Eigen::Affine3d T_support_floatingIdle = Eigen::Affine3d::Identity();
    Eigen::Affine3d T_support_center = Eigen::Affine3d::Identity();

    // Expressing the target (for current flying foot) in the support foot frame
    Eigen::Affine3d T_support_target =
        T_world_support.inverse() *
        ((support_side == Left) ? T_world_targetRight : T_world_targetLeft);

    if (support_side == Left) {
      T_support_floatingIdle.translation().y() = -feet_spacing;
      T_support_center.translation().y() = -feet_spacing / 2.;
    } else {
      T_support_floatingIdle.translation().y() = feet_spacing;
      T_support_center.translation().y() = feet_spacing / 2.;
    }

    // Updating the position
    Eigen::Vector3d error =
        T_support_target.translation() - T_support_floatingIdle.translation();

    double rescale = 1.;

    if (error.x() < -accessibility_length) {
      rescale = std::min(rescale, -accessibility_length / error.x());
      arrived = false;
    }
    if (error.x() > accessibility_length) {
      rescale = std::min(rescale, accessibility_length / error.x());
      arrived = false;
    }
    if (error.y() < -accessibility_width) {
      rescale = std::min(rescale, -accessibility_width / error.y());
      arrived = false;
    }
    if (error.y() > accessibility_width) {
      rescale = std::min(rescale, accessibility_width / error.y());
      arrived = false;
    }

    double dist = error.norm();
    error = error * rescale;

    // Updating the yaw
    double error_yaw;

    if (dist > place_threshold) {
      Eigen::Vector3d target_to_center =
          (T_world_support.inverse() * T_world_target).translation() -
          T_support_center.translation();
      error_yaw = atan2(target_to_center.y(), target_to_center.x());
    } else {
      error_yaw = placo::frame_yaw(T_support_target.rotation());
    }

    if (error_yaw < -accessibility_yaw) {
      arrived = false;
      error_yaw = -accessibility_yaw;
    }
    if (error_yaw > accessibility_yaw) {
      arrived = false;
      error_yaw = accessibility_yaw;
    }

    // Computing new frame
    Eigen::Affine3d new_step;
    new_step.translation() = T_support_floatingIdle.translation() + error;
    new_step.linear() = Eigen::AngleAxisd(error_yaw, Eigen::Vector3d::UnitZ())
                            .toRotationMatrix();

    // Going to next step
    FootstepsPlanner::Footstep footstep(foot_width, foot_length);
    footstep.side = (support_side == Side::Left) ? Side::Right : Side::Left;
    footstep.frame = T_world_support * new_step;
    footsteps.push_back(footstep);

    if (support_side == Side::Left) {
      right_arrived = arrived;
      T_world_currentRight = footstep.frame;
      support_side = Side::Right;
    } else {
      left_arrived = arrived;
      T_world_currentLeft = footstep.frame;
      support_side = Side::Left;
    }
  }

  return footsteps;
}

/**
 * @brief Make sequential double / single support phases from a footsteps plan
 *
 * @param footsteps List of footsteps to reach the target
 * @return Lists of footsteps alternating between single and double support
 * phases
 */
std::vector<FootstepsPlanner::Support>
FootstepsPlanner::make_supports(const std::vector<Footstep> &footsteps) {
  std::vector<FootstepsPlanner::Support> supports;

  // // Building current (double) support
  // Footstep left_footstep;
  // Eigen::Affine3d left_initial;
  // left_initial.translation() = initial_pos.translation();
  // left_initial.translation().y() += foots_spacing;
  // left_initial.linear() = initial_pos.linear();

  // left_footstep.frame = configuration_foots.initial_left_foot;
  // left_footstep.side = Side::Left;

  // Footstep right_footstep;
  // right_footstep.frame = configuration_foots.initial_right_foot;
  // right_footstep.side = Side::Right;

  // Support current_support;
  // current_support.footsteps.push_back(left_footstep);
  // current_support.footsteps.push_back(right_footstep);
  // supports.push_back(current_support);

  // // Previous footstep
  // Footstep last_footstep = left_footstep;
  // if (side_support == Side::Right) {
  //   last_footstep = right_footstep;
  // }

  // for (auto &footstep : footsteps) {
  //   // The last footstep is now a support, the other leg is flying
  //   Support single_phase;
  //   single_phase.footsteps.push_back(last_footstep);
  //   supports.push_back(single_phase);

  //   // The new foostep is on the ground, a double support phase is occurring
  //   Support double_phase;
  //   double_phase.footsteps.push_back(last_footstep);
  //   double_phase.footsteps.push_back(footstep);
  //   supports.push_back(double_phase);

  //   last_footstep = footstep;
  // }

  // return supports;
}
} // namespace placo