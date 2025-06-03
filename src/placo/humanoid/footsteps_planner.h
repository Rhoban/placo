#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include "placo/humanoid/humanoid_robot.h"
#include "placo/humanoid/humanoid_parameters.h"

namespace placo::humanoid
{
class FootstepsPlanner
{
public:
  /**
   * @brief A footstep is the position of a specific foot on the ground
   */
  struct Footstep
  {
    Footstep(double foot_width, double foot_length);

    double foot_width;
    double foot_length;

    HumanoidRobot::Side side;
    Eigen::Affine3d frame;

    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;

    std::vector<Eigen::Vector2d> support_polygon();
    std::vector<Eigen::Vector2d> compute_polygon(double margin = 0.);

    bool operator==(const Footstep& other);

    bool overlap(Footstep& other, double margin = 0.);

    static bool polygon_contains(std::vector<Eigen::Vector2d>& polygon, Eigen::Vector2d point);
  };

  /**
   * @brief A support is a set of footsteps (can be one or two foot on the
   * ground)
   */
  struct Support
  {
    Support();
    Support(std::vector<Footstep> footsteps);

    std::vector<Footstep> footsteps;

    // Time at which the support starts. Is set to -1 if not initialized
    double t_start = -1.;

    // Elapsed ratio of the support phase, ranging from 0 to 1
    double elapsed_ratio = 0.;

    // Time ratio for the remaining part of the support phase
    double time_ratio = 1.;

    // Target DCM in the world frame at the end of the support phase
    Eigen::Vector2d target_world_dcm = Eigen::Vector2d::Zero();

    bool start = false;
    bool end = false;

    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;

    std::vector<Eigen::Vector2d> support_polygon();

    /**
     * @brief Returns the frame for the support. It will be the (interpolated)
     * average of footsteps frames
     * @return a frame
     */
    Eigen::Affine3d frame();

    /**
     * @brief Returns the frame for a given side (if present)
     * @param side the side we want the frame (left or right foot)
     * @return a frame
     */
    Eigen::Affine3d footstep_frame(HumanoidRobot::Side side);

    void apply_offset(Eigen::Vector2d offset);

    bool operator==(const Support& other);

    /**
     * @brief The support side (you should call is_both() to be sure it's not a double support before)
     */
    HumanoidRobot::Side side();

    /**
     * @brief Checks whether this support is a double support
     */
    bool is_both();

    /**
     * @brief Apply a transformation to a support (applied to all the footstep frames)
     */
    friend Support operator*(Eigen::Affine3d T, const Support& support);
  };

  /**
   * @brief Initializes the solver
   * @param parameters Parameters of the walk
   */
  FootstepsPlanner(HumanoidParameters& parameters);

  /**
   * @brief Generate the footsteps
   * @param flying_side first step side
   * @param T_world_left frame of the initial left foot
   * @param T_world_right frame of the initial right foot
   */
  std::vector<Footstep> plan(HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_left,
                             Eigen::Affine3d T_world_right);

  /**
   * @brief Generate the supports from the footsteps
   * @param start should we add a double support at the begining of the move?
   * @param middle should we add a double support between each step ?
   * @param end should we add a double support at the end of the move?
   * @return vector of supports to use. It starts with initial double supports,
   * and add double support phases between footsteps.
   */
  static std::vector<Support> make_supports(std::vector<Footstep> footsteps, double t_start, bool start = true,
                                            bool middle = false, bool end = true);

  /**
   * @brief Return the type of footsteps planner
   */
  virtual std::string name() = 0;

  /**
   * @brief Return the opposite footstep in a neutral position (i.e. at a
   * distance parameters.feet_spacing from the given footstep)
   */
  Footstep opposite_footstep(Footstep footstep, double d_x = 0., double d_y = 0., double d_theta = 0.);

  /**
   * @brief Same as opposite_footstep(), but the clipping is applied
   */
  Footstep clipped_opposite_footstep(Footstep footstep, double d_x = 0., double d_y = 0., double d_theta = 0.);

  Footstep create_footstep(HumanoidRobot::Side side, Eigen::Affine3d T_world_foot);

  // Humanoid parameters for planning and control
  HumanoidParameters& parameters;

protected:
  virtual void plan_impl(std::vector<Footstep>&, HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_left,
                         Eigen::Affine3d T_world_right) = 0;
};
}  // namespace placo::humanoid