#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"

namespace placo
{
void WalkPatternGenerator::plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right)
{
  // Retrieving current foot frames
  auto T_world_leftCurrent = robot.get_T_world_left();
  auto T_world_rightCurrent = robot.get_T_world_right();

  // TODO
}
}  // namespace placo