#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/utils.h"

namespace placo
{
WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot) : robot(robot)
{
}
void WalkPatternGenerator::planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left,
                                         Eigen::Affine3d T_world_right)
{
  // Retrieving current foot frames
  auto T_world_leftCurrent = flatten_on_floor(robot.get_T_world_left());
  auto T_world_rightCurrent = flatten_on_floor(robot.get_T_world_right());

  // Creates the planner and run the planning
  FootstepsPlannerNaive planner(robot.support_side, T_world_leftCurrent, T_world_rightCurrent);
  auto footsteps = planner.plan(T_world_left, T_world_right);

  trajectory.footsteps = planner.make_double_supports(footsteps, true, false, true);
}

void WalkPatternGenerator::planCoM(Trajectory& trajectory)
{
  // Computing how many steps are required
  int ssp_steps = std::round(single_support_duration / dt);
  int dsp_steps = std::round(double_support_duration / dt);
  int total_steps = 0;

  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1)
    {
      total_steps += ssp_steps;
    }
    else
    {
      total_steps += dsp_steps;
    }
  }

  // Creating the planner
  JerkPlanner planner(total_steps, Eigen::Vector2d(0., 0.), Eigen::Vector2d(0., 0.), Eigen::Vector2d(0., 0.), dt,
                      omega);

  // Adding constraints
  int steps = 0;
  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1)
    {
      // Adding a constraint at the begining of the stem
      planner.add_polygon_constraint(steps, support.support_polygon(), JerkPlanner::ZMP, 0.05);

      steps += ssp_steps;
    }
    else
    {
      steps += dsp_steps;
    }

    // We reach the target with the given position, a null speed and a null acceleration
    if (steps >= total_steps)
    {
      auto frame = support.frame();
      planner.add_equality_constraint(
          total_steps - 1, Eigen::Vector2d(frame.translation().x(), frame.translation().y()), JerkPlanner::Position);
      planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Velocity);
      planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Acceleration);
    }
  }

  trajectory.com = planner.plan();
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right)
{
  Trajectory trajectory;

  planFootsteps(trajectory, T_world_left, T_world_right);
  planCoM(trajectory);

  return trajectory;
}
}  // namespace placo