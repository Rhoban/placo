#include "footsteps_planner_repetitive.h"
#include "placo/utils.h"

namespace placo
{
FootstepsPlannerRepetitive::FootstepsPlannerRepetitive(HumanoidParameters& parameters) : FootstepsPlanner(parameters)
{
}

std::vector<FootstepsPlanner::Footstep> FootstepsPlannerRepetitive::plan(HumanoidRobot::Side flying_side,
                                                                         Eigen::Affine3d T_world_left,
                                                                         Eigen::Affine3d T_world_right)
{
  std::vector<FootstepsPlanner::Footstep> footsteps;

  // Including initial footsteps
  auto current_side = flying_side;
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

  if (nb_steps > 0)
  {
    int steps = 0;
    while (steps < nb_steps - 1)
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
  }

  return footsteps;
}

void FootstepsPlannerRepetitive::configure(double x, double y, double theta, int steps)
{
  // XXX: Les x, y et d_theta peuvent aussi être négatifs...
  d_x = (x > max_d_x) ? max_d_x : x;
  d_y = (y > max_d_y) ? max_d_y : y;
  d_theta = (theta > max_d_theta) ? max_d_theta : theta;

  nb_steps = steps;
}
}  // namespace placo