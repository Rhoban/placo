#pragma once

#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"
#include "placo/planning/jerk_planner.h"
#include "rhoban_utils/spline/poly_spline.h"
#include "rhoban_utils/spline/poly_spline_3d.h"

namespace placo
{
class WalkPatternGenerator
{
public:
  /**
   * @brief The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
   */
  HumanoidParameters parameters;

  struct Trajectory
  {
    // Planned footsteps
    std::vector<FootstepsPlanner::Support> footsteps;

    // CoM trajectory
    JerkPlanner::JerkTrajectory2D com;

    // Feet trajectory
    rhoban_utils::PolySpline3D left_foot;
    rhoban_utils::PolySpline left_foot_yaw;
    rhoban_utils::PolySpline3D right_foot;
    rhoban_utils::PolySpline right_foot_yaw;

    rhoban_utils::PolySpline3D &position(HumanoidRobot::Side side);
    rhoban_utils::PolySpline &yaw(HumanoidRobot::Side side);

    Eigen::Affine3d get_T_world_left(double t);
    Eigen::Affine3d get_T_world_right(double t);

    // Trajectory duration
    double duration = 0.0;
  };

  WalkPatternGenerator(HumanoidRobot& robot);

  void planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);
  void planCoM(Trajectory& trajectory);
  void planFeetTrajctories(Trajectory& trajectory);

  Trajectory plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  HumanoidRobot& robot;
};
}  // namespace placo