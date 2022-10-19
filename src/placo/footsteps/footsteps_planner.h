#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo {
class FootstepsPlanner {
public:
  enum Side {
    Left = 0,
    Right
  }

  // A footstep is the position of one foot on the ground
  struct Footstep {
    Side side;
    Eigen::Affine3d frame;
    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;
    const std::vector<Eigen::Vector2d> &support_polygon();
  };

  // Structure containing the configuration of the feet for planning
  struct FootstepConfiguration {
    Eigen::Affine3d initial_right_foot;
    Eigen::Affine3d initial_left_foot;

    Eigen::Affine3d target_right_foot;
    Eigen::Affine3d target_left_foot;
  };

  // A support can be one or two feet supporting the robot (it is
  // a vector of footsteps)
  struct Support {
    std::vector<Footstep> footsteps;
    std::vector<Eigen::Vector2d> polygon;
    bool computed_polygon = false;
    Eigen::Affine3d frame();
    Eigen::Affine3d frame(Side);
    const std::vector<Eigen::Vector2d> &support_polygon();
  };

  FootstepsPlanner(Eigen::Affine3d T_world_right, double foots_spacing);

  // Plans footsteps to bring the robot to the target position and orientation
  // This returns a vector of footsteps indicating where which foot will be
  // placed on the ground sequentially The starting (current) configuration is
  // not included here
  std::vector<Footstep> plan(Eigen::Affine3d target, bool debug = false);
  std::vector<Footstep> plan(Eigen::Affine3d target_left,
                             Eigen::Affine3d target_right, bool debug = false);
  std::vector<Footstep> plan(Side side, Eigen::Affine3d frame,
                             bool debug = false);

  // Make sequential double / single support phases from a footsteps plan
  // The starting (current) configuration is included as the first item
  std::vector<Support> make_supports(const std::vector<Footstep> &footsteps);

  void set_initial(Eigen::Vector3d initial);

  // Defines the steps of the 2 feet ( put -1 in y to place the left foot at a
  // footstep_spacing distance from the right )
  void set_initial(Eigen::Vector3d initial_right, Eigen::Vector3d initial_left);
  void set_support(Side side);

  double foots_spacing;

protected:
  Eigen::Affine3d initial_pos;
  FootstepConfiguration configuration_foots;

  Side side_support = Left;
};
} // namespace placo