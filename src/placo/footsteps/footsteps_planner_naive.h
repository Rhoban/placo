#pragma once

#include "footsteps_planner.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo {
class FootstepsPlannerNaive : public FootstepsPlanner {
public:
  FootstepsPlannerNaive(Side initial_side, Eigen::Affine3d T_world_left,
                        Eigen::Affine3d T_world_right, double feet_spacing);

  FootstepsPlannerNaive(std::string initial_side, Eigen::Affine3d T_world_left,
                        Eigen::Affine3d T_world_right, double feet_spacing);

  /**
   * @see FootstepPlanner::plan(). This is a naive implementation of this
   * problem.
   */
  virtual std::vector<Footstep> plan(Eigen::Affine3d T_world_targetLeft,
                                     Eigen::Affine3d T_world_targetRight);

  // Maximum steps to plan
  int max_steps = 100;

  // Dimension of the accessibility window for the opposite foot
  double accessibility_width = 0.05;
  double accessibility_length = 0.1;
  double accessibility_yaw = 0.2;

  // Distance where the robot walks forward instead of aligning with target
  double place_threshold = 0.5;
};
} // namespace placo