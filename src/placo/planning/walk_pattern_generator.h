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

  struct Trajectory
  {
    double com_height;
    double trunk_pitch;

    // Planned footsteps
    std::vector<FootstepsPlanner::Support> footsteps;

    // CoM trajectory
    JerkPlanner::JerkTrajectory2D com;

    // Feet trajectory
    std::vector<SwingFoot> left_foot;
    rhoban_utils::PolySpline left_foot_yaw;
    rhoban_utils::PolySpline left_foot_tilt;
    std::vector<SwingFoot> right_foot;
    rhoban_utils::PolySpline right_foot_yaw;
    rhoban_utils::PolySpline right_foot_tilt;
    rhoban_utils::PolySpline trunk_yaw;

    std::vector<SwingFoot> &swing_foot(HumanoidRobot::Side side);
    rhoban_utils::PolySpline& yaw(HumanoidRobot::Side side);
    rhoban_utils::PolySpline& tilt(HumanoidRobot::Side side);

    Eigen::Affine3d get_T_world_left(double t);
    Eigen::Affine3d get_T_world_right(double t);
    Eigen::Vector3d get_CoM_world(double t);
    Eigen::Matrix3d get_R_world_trunk(double t);

    // Trajectory duration
    double duration = 0.0;
  };

  WalkPatternGenerator(HumanoidRobot& robot);

  void planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right,
                     Eigen::Affine3d T_world_targetLeft, Eigen::Affine3d T_world_targetRight);
  void planCoM(Trajectory& trajectory);
  void planFeetTrajctories(Trajectory& trajectory);

  Trajectory plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Affine3d T_world_targetLeft,
                  Eigen::Affine3d T_world_targetRight);
  Trajectory plan(std::vector<FootstepsPlanner::Support> footsteps);

  // For python binding
  Trajectory plan_by_frames(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right,
                            Eigen::Affine3d T_world_targetLeft, Eigen::Affine3d T_world_targetRight);

  HumanoidRobot& robot;
};
}  // namespace placo