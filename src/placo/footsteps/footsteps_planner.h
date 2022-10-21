#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo {
class FootstepsPlanner {
public:
  /**
   * @brief Which side the foot is
   */
  enum Side { Left = 0, Right };

  /**
   * @brief A footstep is the position of a specific foot on the ground
   */
  struct Footstep {
    Footstep(double foot_width, double foot_length);
    double foot_width;
    double foot_length;
    Side side;
    Eigen::Affine3d frame;
    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;

    bool operator==(const Footstep &other);
    
    std::vector<Eigen::Vector2d> support_polygon();
  };

  // A support can be one or two feet supporting the robot (it is
  // a vector of footsteps)
  struct Support {
    std::vector<Footstep> footsteps;
    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;
    Eigen::Affine3d frame();
    Eigen::Affine3d frame(Side);
    std::vector<Eigen::Vector2d> support_polygon();

    bool operator==(const Support &other);
  };

  FootstepsPlanner(Side initial_side, Eigen::Affine3d T_world_left,
                   Eigen::Affine3d T_world_right, double feet_spacing);

  FootstepsPlanner(std::string initial_side, Eigen::Affine3d T_world_left,
                   Eigen::Affine3d T_world_right, double feet_spacing);

  // Plans footsteps to bring the robot to the target position and orientation
  // This returns a vector of footsteps indicating where which foot will be
  // placed on the ground sequentially The starting (current) configuration is
  // not included here
  std::vector<Footstep> plan(Eigen::Affine3d T_world_targetLeft,
                             Eigen::Affine3d T_world_targetRight);

  // Make sequential double / single support phases from a footsteps plan
  // The starting (current) configuration is included as the first item
  std::vector<Support> make_double_supports(const std::vector<Footstep> &footsteps);

  // Maximum steps to plan
  int max_steps = 100;

  // Dimension of the accessibility window for the opposite foot
  double accessibility_width = 0.05;
  double accessibility_length = 0.1;
  double accessibility_yaw = 0.2;

  // Distance where the robot walks forward instead of aligning with target
  double place_threshold = 0.5;

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
} // namespace placo