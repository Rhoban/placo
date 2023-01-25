#pragma once

#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"
#include "placo/planning/jerk_planner.h"
#include "rhoban_utils/spline/poly_spline.h"
#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/planning/swing_foot.h"

namespace placo
{
class WalkPatternGenerator
{
public:
  /**
   * @brief The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
   */
  HumanoidParameters parameters;

  struct TrajectoryPart
  {
    SwingFoot::Trajectory swing_trajectory;
    double t_start;
    double t_end;

    FootstepsPlanner::Support support;
  };

  struct Trajectory
  {
    double com_height;
    double trunk_pitch;

    // Planned footsteps
    std::vector<FootstepsPlanner::Support> footsteps;

    // A part is the support and the swing trajectory
    std::vector<TrajectoryPart> parts;

    // CoM trajectory
    JerkPlanner::JerkTrajectory2D com;

    // Feet trajectory
    rhoban_utils::PolySpline left_foot_yaw;
    rhoban_utils::PolySpline right_foot_yaw;
    rhoban_utils::PolySpline trunk_yaw;

    rhoban_utils::PolySpline& yaw(HumanoidRobot::Side side);

    Eigen::Affine3d get_T_world_left(double t);
    Eigen::Affine3d get_T_world_right(double t);
    Eigen::Vector3d get_CoM_world(double t);
    Eigen::Matrix3d get_R_world_trunk(double t);

    HumanoidRobot::Side support_side(double t);

    std::vector<Eigen::Affine3d> get_last_footsteps(double t, bool double_support);
    Eigen::Affine3d get_last_footstep(double t, bool double_support);
    Eigen::Affine3d get_last_last_footstep(double t, bool double_support);

    // Trajectory duration
    double duration = 0.0;

    // Number of steps planned by the jerk planner
    int jerk_planner_steps;
  };

  WalkPatternGenerator(HumanoidRobot& robot);

  void planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right,
                     Eigen::Affine3d T_world_targetLeft, Eigen::Affine3d T_world_targetRight);
  void planCoM(Trajectory& trajectory);
  void planFeetTrajectories(Trajectory& trajectory);

  Trajectory plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Affine3d T_world_targetLeft,
                  Eigen::Affine3d T_world_targetRight);
  Trajectory plan(std::vector<FootstepsPlanner::Support> footsteps);

  // For python binding
  Trajectory plan_by_frames(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right,
                            Eigen::Affine3d T_world_targetLeft, Eigen::Affine3d T_world_targetRight);
  Trajectory plan_by_supports(std::vector<FootstepsPlanner::Support> footsteps);

  HumanoidRobot& robot;
};
}  // namespace placo