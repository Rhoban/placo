#pragma once

#include "placo/model/mobile_robot.h"

namespace placo
{
class LeggedRobot : public MobileRobot
{
public:
  LeggedRobot(std::string model_directory = "robot/");

  /**
   * @brief Updates which frame should be the current support
   */
  void update_support(MobileRobot::FrameIndex new_support_frame);
  void update_support(const std::string &frame);

  // We suppose we have one support frame and associated transformation
  MobileRobot::FrameIndex support_frame;
  Eigen::Affine3d T_world_support;
};
}  // namespace placo