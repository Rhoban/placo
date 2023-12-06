#pragma once

#include "placo/humanoid/footsteps_planner.h"
#include "placo/humanoid/humanoid_robot.h"
#include "placo/humanoid/humanoid_parameters.h"
#include "placo/tools/cubic_spline_3d.h"
#include "placo/humanoid/swing_foot_cubic.h"
#include "placo/humanoid/swing_foot.h"
#include "placo/humanoid/kick.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/kinematics/frame_task.h"
#include "placo/kinematics/com_task.h"
#include "placo/kinematics/orientation_task.h"
#include "placo/problem/integrator.h"
#include "placo/problem/problem.h"
#include "placo/humanoid/lipm.h"

namespace placo::humanoid
{
class WalkPatternGenerator
{
public:
  struct TrajectoryPart
  {
    double t_start;
    double t_end;

    bool kick_part = false;
    SwingFootCubic::Trajectory swing_trajectory;
    Kick::KickTrajectory kick_trajectory;

    FootstepsPlanner::Support support;
  };

  struct Trajectory
  {
    Trajectory();

    double trunk_pitch = 0.;
    double trunk_roll = 0.;

    double com_target_z;

    int kept_ts = 0;

    Eigen::Affine3d get_T_world_left(double t);
    Eigen::Affine3d get_T_world_right(double t);
    Eigen::Vector3d get_v_world_left(double t);
    Eigen::Vector3d get_v_world_right(double t);
    Eigen::Affine3d get_T_world_foot(HumanoidRobot::Side side, double t);

    Eigen::Vector3d get_p_world_CoM(double t);
    Eigen::Vector3d get_v_world_CoM(double t);
    Eigen::Vector3d get_a_world_CoM(double t);
    Eigen::Vector3d get_j_world_CoM(double t);

    Eigen::Vector2d get_p_world_DCM(double t, double omega);
    Eigen::Vector2d get_p_world_ZMP(double t, double omega);

    Eigen::Matrix3d get_R_world_trunk(double t);

    HumanoidRobot::Side support_side(double t);
    bool support_is_both(double t);
    bool is_flying(HumanoidRobot::Side side, double t);

    /**
     * @brief Returns the support corresponding to the given time in the trajectory
    */
    FootstepsPlanner::Support get_support(double t);

    /**
     * @brief Returns the nth next support corresponding to the given time in the trajectory
     */
    FootstepsPlanner::Support get_next_support(double t, int n=1);

    /**
     * @brief Returns the nth previous support corresponding to the given time in the trajectory
     */
    FootstepsPlanner::Support get_prev_support(double t, int n=1);

    std::vector<FootstepsPlanner::Support> get_supports();
    int remaining_supports(double t);

    /**
     * @brief Applies a given transformation to the left of all values issued by the trajectory
     */
    void apply_transform(Eigen::Affine3d T);

    // Trajectory duration
    double t_start = 0.0;
    double t_end = 0.0;

    /**
     * @brief Returns the trajectory time start for the support corresponding to the given time
     */
    double get_part_t_start(double t);

    /**
     * @brief Returns the trajectory time end for the support corresponding to the given time
     */
    double get_part_t_end(double t);

    // Number of dt planned by the jerk planner
    int jerk_planner_timesteps = 0;

  protected:
    /**
     * @brief Retrieves the yaw value
     */
    placo::tools::CubicSpline& yaw(HumanoidRobot::Side side);

    // Planned supports
    std::vector<FootstepsPlanner::Support> supports;

    // A part is the support and the swing trajectory
    std::vector<TrajectoryPart> parts;

    // CoM trajectories
    LIPM::Trajectory com;

    // Feet trajectory
    placo::tools::CubicSpline left_foot_yaw;
    placo::tools::CubicSpline right_foot_yaw;
    placo::tools::CubicSpline trunk_yaw;

    void add_supports(double t, FootstepsPlanner::Support& support);

    /**
     * @brief A (left) transformation to apply to all the outputs
     */
    Eigen::Affine3d T;

    /**
     * @brief WalkPatternGenerator is allowed to access the protected fields in the trajectory when building it
     */
    friend class WalkPatternGenerator;
  };

  WalkPatternGenerator(HumanoidRobot& robot, HumanoidParameters& parameters);

  /**
   * @brief Plan a walk trajectory following given footsteps based on the parameters of the WPG
   * @param supports Supports generated from the foosteps to follow
   * @return Planned trajectory
   */
  Trajectory plan(std::vector<FootstepsPlanner::Support>& supports, Eigen::Vector3d initial_com_world,
                  double t_start = 0.);

  /**
   * @brief Update the walk trajectory to follow given footsteps based on the parameters of the WPG.
   * @param supports Supports generated from the current foosteps or the new
   * ones to follow. Contain the current support
   * @param old_trajectory Current walk trajectory
   * @param t_replan The time (in the original trajectory) where the replan happens
   * @return True if the trajectory have been replanned, false it hasn't
   */
  Trajectory replan(std::vector<FootstepsPlanner::Support>& supports, Trajectory& old_trajectory, double t_replan);

  /**
   * @brief Checks if a trajectory can be replanned for supports
   */
  bool can_replan_supports(Trajectory& trajectory, double t_replan);

  /**
   * @brief Replan the supports for a given trajectory given a footsteps planner
   */
  std::vector<FootstepsPlanner::Support> replan_supports(FootstepsPlanner& planner, Trajectory& trajectory,
                                                         double t_replan);

protected:
  // Robot associated to the WPG
  HumanoidRobot& robot;

  // The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
  HumanoidParameters& parameters;

  double omega;
  double omega_2;

  void planCoM(Trajectory& trajectory, Eigen::Vector2d initial_pos,
               Eigen::Vector2d initial_vel = Eigen::Vector2d::Zero(),
               Eigen::Vector2d initial_acc = Eigen::Vector2d::Zero(), Trajectory* old_trajectory = nullptr,
               double t_replan = 0.);

  void planFeetTrajectories(Trajectory& trajectory, Trajectory* old_trajectory = nullptr, double t_replan = 0.);

  void planKickTrajectory(TrajectoryPart& part, Trajectory& trajectory, int step, double& t);
  void planDoubleSupportTrajectory(TrajectoryPart& part, Trajectory& trajectory, double& t);
  void planSingleSupportTrajectory(TrajectoryPart& part, Trajectory& trajectory, int step, double& t,
                                   Trajectory* old_trajectory, double t_replan);

  int support_timesteps(FootstepsPlanner::Support& support);
};
}  // namespace placo::humanoid