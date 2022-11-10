#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo
{
class FootstepsPlanner
{
public:
  /**
   * @brief Which side the foot is
   */
  enum Side
  {
    Left = 0,
    Right
  };

  /**
   * @brief A footstep is the position of a specific foot on the ground
   */
  struct Footstep
  {
    Footstep(double foot_width, double foot_length);
    double foot_width;
    double foot_length;
    Side side;
    Eigen::Affine3d frame;
    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;

    bool operator==(const Footstep& other);

    std::vector<Eigen::Vector2d> support_polygon();
  };

  /**
   * @brief A support is a set of footsteps (can be one or two foot on the
   * ground)
   */
  struct Support
  {
    std::vector<Footstep> footsteps;
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
    Eigen::Affine3d frame(Side side);

    bool operator==(const Support& other);
  };

  /**
   * @brief Initializes the solver
   * @param initial_side side that is initially supporting the robot
   * @param T_world_left frame of the initial left foot
   * @param T_world_right frame of the initial right foot
   * @param feet_spacing spacing between feet
   */
  FootstepsPlanner(Side initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, double feet_spacing);

  /**
   * @brief This constructors allow the initial_side to be a string (useful for
   * Python bindings)
   */
  FootstepsPlanner(std::string initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right,
                   double feet_spacing);

  /**
   * @brief Plan the footsteps
   * @param T_world_targetLeft target frame for left foot
   * @param T_world_targetRight target frame for right foot
   * @return vector of footsteps to apply. It starts with initial footsteps
   * (the first is the current flying foot, the second the current support foot)
   * and ends with the footsteps that reached the given target
   */
  virtual std::vector<Footstep> plan(Eigen::Affine3d T_world_targetLeft, Eigen::Affine3d T_world_targetRight) = 0;

  /**
   * @brief From planned footsteps, this method adds the double support phases
   * @param footsteps a vector of footsteps ad produces by plan
   * @return vector of supports to use. It starts with initial double supports,
   * and add double support phases between footsteps.
   */
  std::vector<Support> make_double_supports(const std::vector<Footstep>& footsteps);

  // Foot dimensions
  double foot_width = 0.1;
  double foot_length = 0.15;

protected:
  // Frames for initial and target feet placements
  Side initial_side;
  Eigen::Affine3d T_world_left;
  Eigen::Affine3d T_world_right;
  double feet_spacing;
};
}  // namespace placo