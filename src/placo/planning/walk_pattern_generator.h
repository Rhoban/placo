#pragma once

#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"
#include "placo/planning/solver_task_holder.h"
#include "placo/planning/jerk_planner.h"
#include "rhoban_utils/spline/poly_spline.h"
#include "rhoban_utils/spline/poly_spline_3d.h"
#include "placo/planning/swing_foot.h"
#include "placo/control/kinematics_solver.h"
#include "placo/control/frame_task.h"
#include "placo/control/com_task.h"
#include "placo/control/orientation_task.h"

namespace placo
{
class WalkPatternGenerator
{
public:
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

    // Planned supports
    std::vector<FootstepsPlanner::Support> supports;

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

    /// @brief Return the frame of the last left or right footstep before a certain moment in the trajectory
    /// @param side Side of the footstep
    /// @param t Moment in the trajectory
    /// @return Frame of the footstep
    Eigen::Affine3d get_last_footstep_frame(HumanoidRobot::Side side, double t);

    // Trajectory duration
    double duration = 0.0;

    // Number of steps planned by the jerk planner
    int jerk_planner_steps;
  };

  WalkPatternGenerator(HumanoidRobot& robot, FootstepsPlanner& footsteps_planner, HumanoidParameters& parameters);

  /// @brief Plan a trajectory based on the footsteps planner and the parameters of the WPG
  Trajectory plan();

  /// @brief Replan a trajectory adapted to the previous one
  /// @param trajectory Previous trajectory
  /// @param elapsed_time Elapsed time on the previous trajectory
  Trajectory replan(Trajectory& previous_trajectory, double elapsed_time);

protected:
  // Robot associated to the WPG
  HumanoidRobot& robot;

  // Planner used to generate the footsteps
  FootstepsPlanner& footsteps_planner;

  // The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
  HumanoidParameters& parameters;

  void planCoM(Trajectory& trajectory, Eigen::Vector2d initial_vel = Eigen::Vector2d::Zero(),
               Eigen::Vector2d initial_acc = Eigen::Vector2d::Zero());

  void planFeetTrajectories(Trajectory& trajectory);
};
}  // namespace placo