#pragma once

#include "placo/humanoid/footsteps_planner.h"
#include "placo/humanoid/humanoid_robot.h"
#include "placo/humanoid/humanoid_parameters.h"
#include "placo/tools/cubic_spline_3d.h"
#include "placo/humanoid/swing_foot_cubic.h"
#include "placo/humanoid/swing_foot.h"
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
    TrajectoryPart(FootstepsPlanner::Support support, double t_start = 0.);

    double t_start;
    double t_end;

    FootstepsPlanner::Support support;

    LIPM::Trajectory com_trajectory;

    SwingFootCubic::Trajectory swing_trajectory;
  };

  struct Trajectory
  {
    Trajectory();
    Trajectory(double com_target_z, double t_start = 0., double trunk_pitch = 0., double trunk_roll = 0.);

    // Debug
    void print_parts_timings();

    // Trajectory duration
    double t_start;
    double t_end;

    // A part of the trajectory contain a support and the associated trajectories (CoM, swing foot)
    std::vector<TrajectoryPart> parts;

    // Constants of the trajectory
    double com_target_z;
    double trunk_pitch;
    double trunk_roll;

    int kept_ts = 0;

    Eigen::Affine3d get_T_world_left(double t);
    Eigen::Affine3d get_T_world_right(double t);
    Eigen::Affine3d get_T_world_foot(HumanoidRobot::Side side, double t);

    Eigen::Vector3d get_v_world_left(double t);
    Eigen::Vector3d get_v_world_right(double t);
    Eigen::Vector3d get_v_world_foot(HumanoidRobot::Side side, double t);

    double get_yaw_world_left(double t);
    double get_yaw_world_right(double t);
    double get_yaw_world_foot(HumanoidRobot::Side side, double t);
    double get_yaw_world_trunk(double t);

    Eigen::Vector3d get_p_world_CoM(double t);
    Eigen::Vector3d get_v_world_CoM(double t);
    Eigen::Vector3d get_a_world_CoM(double t);
    Eigen::Vector3d get_j_world_CoM(double t);

    Eigen::Vector2d get_p_world_DCM(double t, double omega);
    Eigen::Vector2d get_p_world_ZMP(double t, double omega);

    Eigen::Matrix3d get_R_world_trunk(double t);

    Eigen::Vector3d get_p_support_CoM(double t);
    Eigen::Vector3d get_v_support_CoM(double t);
    Eigen::Vector2d get_p_support_DCM(double t, double omega);

    HumanoidRobot::Side support_side(double t);
    bool support_is_both(double t);
    bool is_flying(HumanoidRobot::Side side, double t);

    /**
     * @brief Returns the support corresponding to the given time in the trajectory
     */
    FootstepsPlanner::Support get_support(double t);

    /**
     * @brief Returns the nth next support corresponding to the given time in the trajectory.
     * If n is greater than the number of remaining supports, the last support is returned
     */
    FootstepsPlanner::Support get_next_support(double t, int n = 1);

    /**
     * @brief Returns the nth previous support corresponding to the given time in the trajectory.
     * If n is greater than the number of previous supports, the first support is returned
     */
    FootstepsPlanner::Support get_prev_support(double t, int n = 1);

    std::vector<FootstepsPlanner::Support> get_supports();
    int remaining_supports(double t);

    /**
     * @brief Applies a given transformation to the left of all values issued by the trajectory
     */
    void apply_transform(Eigen::Affine3d T);

    /**
     * @brief Returns the start time of the trajectory part corresponding to the given time
     */
    double get_part_t_start(double t);

    /**
     * @brief Returns the end time of the trajectory part corresponding to the given time
     */
    double get_part_t_end(double t);

    /**
     * @brief Returns the DCM at the end of the trajectory part corresponding to the given time
     */
    Eigen::Vector2d get_part_end_dcm(double t, double omega);

  protected:
    // Yaw trajectories
    placo::tools::CubicSpline left_foot_yaw;
    placo::tools::CubicSpline right_foot_yaw;
    placo::tools::CubicSpline trunk_yaw;

    /**
     * @brief Retrieves the yaw value of a foot
     */
    placo::tools::CubicSpline& foot_yaw(HumanoidRobot::Side side);

    /**
     * @brief Adds a support to the trajectory
     */
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
   * @brief Plans a walk trajectory following given footsteps based on the parameters of the WPG
   * @param supports Supports generated from the foosteps to follow
   * @return Planned trajectory
   */
  Trajectory plan(std::vector<FootstepsPlanner::Support>& supports, Eigen::Vector3d initial_com_world,
                  double t_start = 0.);

  /**
   * @brief Updates the walk trajectory to follow given footsteps based on the parameters of the WPG.
   * @param supports Supports generated from the current foosteps or the new
   * ones to follow. Contain the current support
   * @param old_trajectory Current walk trajectory
   * @param t_replan The time (in the original trajectory) where the replan happens
   * @return Updated trajectory
   */
  Trajectory replan(std::vector<FootstepsPlanner::Support>& supports, Trajectory& old_trajectory, double t_replan);

  /**
   * @brief Checks if a trajectory can be replanned for supports.
   */
  bool can_replan_supports(Trajectory& trajectory, double t_replan);

  /**
   * @brief Replans the supports for a given trajectory given a footsteps planner.
   */
  std::vector<FootstepsPlanner::Support> replan_supports(FootstepsPlanner& planner, Trajectory& trajectory,
                                                         double t_replan, double t_last_replan);

  /**
   * @brief Updates the supports to ensure DCM viability by adjusting the
   * duration and the target of the current swing trajectory.
   * @param t Current time
   * @param supports Planned supports
   * @param world_measured_dcm Measured DCM in world frame
   */
  std::vector<FootstepsPlanner::Support> update_supports(double t, std::vector<FootstepsPlanner::Support> supports,
                                                         Eigen::Vector2d world_measured_dcm);

  /**
   * @brief Computes the best ZMP in the support polygon to move de DCM from
   * world_dcm_start to world_dcm_end in duration.
   * @param world_dcm_start Initial DCM position in world frame
   * @param world_dcm_end Desired final DCM position in world frame
   * @param duration Duration
   * @param support Support
   */
  Eigen::Vector2d get_optimal_zmp(Eigen::Vector2d world_dcm_start, Eigen::Vector2d world_dcm_end, double duration,
                                  FootstepsPlanner::Support& support);

  int support_default_timesteps(FootstepsPlanner::Support& support);
  double support_default_duration(FootstepsPlanner::Support& support);

  bool soft = false;
  double zmp_in_support_weight = 1e3;
  double stop_end_support_weight = 1e3;

protected:
  // Robot associated to the WPG
  HumanoidRobot& robot;

  // The parameters to use for planning. The values are forwarded to the relevant solvers when needed.
  HumanoidParameters& parameters;

  // Natural frequency of the LIPM (omega = sqrt(g/h))
  double omega;

  // Squared natural frequency of the LIPM (omega^2 = g/h)
  double omega_2;

  /**
   * @brief Constrains the LIPM to ensure that the ZMP stays in the support polygon and that the CoM stops at
   * the end of an end support.
   * @param problem Problem to add the constraints to
   * @param lipm LIPM to constrain
   * @param support Support to constrain
   * @param omega_2 Squared natural frequency of the LIPM (omega^2 = g/h)
   * @param parameters Humanoid parameters to use for the constraints
   */
  void constrain_lipm(problem::Problem& problem, LIPM& lipm, FootstepsPlanner::Support& support, double omega_2,
                      HumanoidParameters& parameters);

  /**
   * @brief Plans the CoM trajectory for a given support. Returns false if the QP solver failed to solve the problem.
   * @param trajectory Trajectory to fill
   * @param support Support to plan
   * @param initial_pos Initial position of the CoM in the world frame
   * @param initial_vel Initial velocity of the CoM in the world frame
   * @param initial_acc Initial acceleration of the CoM in the world frame
   */
  bool plan_com(Trajectory& trajectory, std::vector<FootstepsPlanner::Support>& supports, Eigen::Vector2d initial_pos,
                Eigen::Vector2d initial_vel = Eigen::Vector2d::Zero(),
                Eigen::Vector2d initial_acc = Eigen::Vector2d::Zero());

  void plan_dbl_support(Trajectory& trajectory, int part_index);
  void plan_sgl_support(Trajectory& trajectory, int part_index, Trajectory* old_trajectory);
  void plan_feet_trajectories(Trajectory& trajectory, Trajectory* old_trajectory = nullptr);
};
}  // namespace placo::humanoid