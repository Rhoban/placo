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
    Eigen::Vector3d get_v_world_left(double t);
    Eigen::Vector3d get_v_world_right(double t);

    Eigen::Vector3d get_p_world_CoM(double t);
    Eigen::Matrix3d get_R_world_trunk(double t);

    HumanoidRobot::Side support_side(double t);

    FootstepsPlanner::Support get_support(double t);
    FootstepsPlanner::Support get_next_support(double t);
    FootstepsPlanner::Support get_prev_support(double t);

    double get_phase_t_start(double t);

    // Trajectory duration
    double duration = 0.0;

    // Number of dt planned by the jerk planner
    int jerk_planner_nb_dt = 0;

    // Time offsets
    double time_offset = 0.0;
    double supports_update_offset = 0.0;

    // Can we update the supports ?
    bool are_supports_updatable = false;

    // Initial position of the flying foot of the first support phase
    // Used to ensure continuity of the swing trajectories after a replanning
    Eigen::Affine3d initial_T_world_flying_foot;
  };

  WalkPatternGenerator(HumanoidRobot& robot, HumanoidParameters& parameters);

  /// @brief Plan a walk trajectory following given footsteps based on the parameters of the WPG
  /// @param supports Supports generated from the foosteps to follow
  /// @return Planned trajectory
  Trajectory plan(std::vector<FootstepsPlanner::Support>& supports);

  /// @brief Update the walk trajectory to follow given footsteps based on the parameters of the WPG.
  /// It ensure a continuous CoM trajectory and replan only if replan_frequency dt have passed
  /// @param supports Supports generated from the current foosteps or the new
  /// ones to follow. Contain the current support
  /// @param trajectory Current walk trajectory
  /// @param elapsed Elapsed time following the trajectory
  /// @return True if the trajectory have been replanned, false it hasn't
  bool replan(std::vector<FootstepsPlanner::Support>& supports, Trajectory& trajectory, double elapsed);

  // /// @brief Plan a trajectory adapted to the previous one ending with one foot in the air at a targeted position
  // /// @param previous_trajectory Previous trajectory
  // /// @param elapsed_time Elapsed time on the previous trajectory
  // /// @param kicking_side Side of the foot to put in the air.
  // /// Has to be the next flying foot when prepare_kick() is called
  // /// @param T_world_target Targeted frame for the foot in the air
  // /// @return Planned trajectory
  // Trajectory plan_kick(Trajectory& previous_trajectory, double elapsed_time, HumanoidRobot::Side kicking_side,
  //                      Eigen::Affine3d T_world_target);

  // Trajectory plan_one_foot_balance(Trajectory& previous_trajectory, double elapsed_time,
  //                                  HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_target);

protected:
  // Robot associated to the WPG
  HumanoidRobot& robot;

  // The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
  HumanoidParameters& parameters;

  void planCoM(Trajectory& trajectory, Eigen::Vector2d initial_vel = Eigen::Vector2d::Zero(),
               Eigen::Vector2d initial_acc = Eigen::Vector2d::Zero());

  void planFeetTrajectories(Trajectory& trajectory);

  // Initialize the trajectory
  Trajectory initTrajectory(std::vector<FootstepsPlanner::Support>& supports);

  // Initialize a new trajectory according to a previous one
  Trajectory updateTrajectory(std::vector<FootstepsPlanner::Support>& supports, Trajectory& trajectory, double elapsed);

  // std::vector<FootstepsPlanner::Support> planSupportsKick(Trajectory trajectory, HumanoidRobot::Side kicking_side,
  //                                                         Eigen::Affine3d T_world_left, Eigen::Affine3d
  //                                                         T_world_right);

  // void planCoMKick(Trajectory& trajectory, Eigen::Vector2d initial_vel, Eigen::Vector2d initial_acc);

  // void planCoMOneFoot(Trajectory& trajectory, Eigen::Vector2d initial_vel, Eigen::Vector2d initial_acc);

  // void planFeetKick(Trajectory& trajectory, HumanoidRobot::Side kicking_side, Eigen::Affine3d T_world_target);

  // void planFeetOneFoot(Trajectory& trajectory, HumanoidRobot::Side kicking_side, Eigen::Affine3d T_world_target);
};
}  // namespace placo