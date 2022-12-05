#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/utils.h"

namespace placo
{
void WalkPatternGenerator::planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left,
                                         Eigen::Affine3d T_world_right)
{
  // Retrieving current foot frames
  auto T_world_leftCurrent = flatten_on_floor(robot.get_T_world_left());
  auto T_world_rightCurrent = flatten_on_floor(robot.get_T_world_right());

  // Creates the planner and run the planning
  FootstepsPlannerNaive planner(robot.support_side, T_world_leftCurrent, T_world_rightCurrent);
  trajectory.footsteps = planner.plan(T_world_left, T_world_right);
}

void WalkPatternGenerator::planCoM(Trajectory &trajectory)
{
  // JerkPlanner planner(
}

void WalkPatternGenerator::plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right)
{
  Trajectory trajectory;

  planFootsteps(trajectory, T_world_left, T_world_right);
}
}  // namespace placo