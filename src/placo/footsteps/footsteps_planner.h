#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"

namespace placo
{
class FootstepsPlanner
{
public:
  /**
   * @brief Humanoid parameters for planning and control
   */
  HumanoidParameters parameters;

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
    bool start_end = false;
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
    Eigen::Affine3d frame(HumanoidRobot::Side side);

    bool operator==(const Support& other);

    /**
     * @brief The support side (or Both if it's a double support)
     */
    HumanoidRobot::Side side();
  };

  /**
   * @brief Initializes the solver
   * @param initial_side side that is initially supporting the robot
   * @param T_world_left frame of the initial left foot
   * @param T_world_right frame of the initial right foot
   * @param feet_spacing spacing between feet
   */
  FootstepsPlanner(HumanoidRobot::Side initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  /**
   * @brief This constructors allow the initial_side to be a string (useful for
   * Python bindings)
   */
  FootstepsPlanner(std::string initial_side, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right);

  /**
   * @brief From planned footsteps, this method adds the double support phases
   * @param footsteps a vector of footsteps ad produces by plan
   * @param start should we add a double support at the begining of the move?
   * @param middle should we add a double support between each step ?
   * @param end should we add a double support at the end of the move?
   * @return vector of supports to use. It starts with initial double supports,
   * and add double support phases between footsteps.
   */
  std::vector<Support> make_double_supports(const std::vector<Footstep>& footsteps, bool start = false,
                                            bool middle = false, bool end = false);

protected:
  // Frames for initial and target feet placements
  HumanoidRobot::Side initial_side;
  Eigen::Affine3d T_world_left;
  Eigen::Affine3d T_world_right;
};
}  // namespace placo