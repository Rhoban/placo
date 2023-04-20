#include "footsteps_planner_repetitive.h"
#include "placo/utils.h"

namespace placo
{
FootstepsPlannerRepetitive::FootstepsPlannerRepetitive(HumanoidParameters& parameters) : FootstepsPlanner(parameters)
{
}

void FootstepsPlannerRepetitive::plan_impl(std::vector<FootstepsPlanner::Footstep>& footsteps,
                                           HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_left,
                                           Eigen::Affine3d T_world_right)
{
  // Including initial footsteps
  HumanoidRobot::Side current_side = HumanoidRobot::other_side(flying_side);
  auto T_world_current_frame = (current_side == HumanoidRobot::Side::Left) ? T_world_left : T_world_right;

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
      footsteps.push_back(create_footstep(current_side, T_world_current_frame));

      steps += 1;
    }

    // Adding last footstep to go double support
    T_world_current_frame.translate(Eigen::Vector3d(
        0, (current_side == HumanoidRobot::Side::Left) ? -parameters.feet_spacing : parameters.feet_spacing, 0));
    footsteps.push_back(create_footstep(HumanoidRobot::other_side(current_side), T_world_current_frame));
  }
}

void FootstepsPlannerRepetitive::configure(double x, double y, double theta, int steps)
{
  d_x = x;
  d_y = y;
  d_theta = theta;
  nb_steps = steps;
}
}  // namespace placo