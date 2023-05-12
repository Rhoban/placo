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
  Footstep footstep = footsteps[1];

  if (nb_steps > 0)
  {
    int steps = 0;
    while (steps < nb_steps - 1)
    {
      footstep = opposite_footstep(footstep);
      footstep.frame.translate(Eigen::Vector3d(d_x, d_y, 0));
      footstep.frame.rotate(Eigen::AngleAxisd(d_theta, Eigen::Vector3d(0, 0, 1)));

      footsteps.push_back(footstep);
      steps += 1;
    }

    // Adding last footstep to go double support
    footsteps.push_back(opposite_footstep(footstep));
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