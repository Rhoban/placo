#include "footsteps_planner_repetitive.h"
#include "placo/utils.h"

namespace placo
{
FootstepsPlannerRepetitive::FootstepsPlannerRepetitive(HumanoidRobot::Side initial_side, Eigen::Affine3d T_world_left,
                                                       Eigen::Affine3d T_world_right)
  : FootstepsPlanner(initial_side, T_world_left, T_world_right)
{
}

FootstepsPlannerRepetitive::FootstepsPlannerRepetitive(std::string initial_side, Eigen::Affine3d T_world_left,
                                                       Eigen::Affine3d T_world_right)
  : FootstepsPlanner(initial_side, T_world_left, T_world_right)
{
}

std::vector<FootstepsPlanner::Footstep> FootstepsPlannerRepetitive::plan(double d_x, double d_y, double d_theta,
                                                                         int nb_steps)
{
  std::vector<FootstepsPlanner::Footstep> footsteps;

  // Including initial footsteps
  auto current_side = initial_side;
  auto T_world_current_frame = (current_side == HumanoidRobot::Side::Left) ? T_world_left : T_world_right;
  FootstepsPlanner::Footstep footstep(parameters.foot_width, parameters.foot_length);
  footstep.side = current_side;
  footstep.frame = T_world_current_frame;
  footsteps.push_back(footstep);

  current_side = HumanoidRobot::other_side(current_side);
  T_world_current_frame = (current_side == HumanoidRobot::Side::Left) ? T_world_left : T_world_right;
  footstep.side = current_side;
  footstep.frame = T_world_current_frame;
  footsteps.push_back(footstep);

  // Bounds
  d_x = (d_x > max_d_x) ? max_d_x : d_x;
  d_y = (d_y > max_d_y) ? max_d_y : d_y;
  d_theta = (d_theta > max_d_theta) ? max_d_theta : d_theta;

  int steps = 0;
  while (steps < nb_steps)
  {
    T_world_current_frame.translate(Eigen::Vector3d(
        d_x,
        (current_side == HumanoidRobot::Side::Left) ? d_y - parameters.feet_spacing : d_y + parameters.feet_spacing,
        0));

    T_world_current_frame.rotate(Eigen::AngleAxisd(d_theta, Eigen::Vector3d(0, 0, 1)));

    current_side = HumanoidRobot::other_side(current_side);

    // Adding the footstep
    footstep.side = current_side;
    footstep.frame = T_world_current_frame;
    footsteps.push_back(footstep);

    steps += 1;
  }

  // Adding last footstep to go double support
  footstep.side = HumanoidRobot::other_side(current_side);
  T_world_current_frame.translate(Eigen::Vector3d(
      0, (current_side == HumanoidRobot::Side::Left) ? -parameters.feet_spacing : parameters.feet_spacing, 0));
  footstep.frame = T_world_current_frame;
  footsteps.push_back(footstep);

  return footsteps;
}

std::vector<FootstepsPlanner::Footstep> FootstepsPlannerRepetitive::plan_with_config(double d_x, double d_y,
                                                                                     double d_theta, int nb_steps,
                                                                                     placo::HumanoidParameters config)
{
  parameters = config;
  return FootstepsPlannerRepetitive::plan(d_x, d_y, d_theta, nb_steps);
}
}  // namespace placo