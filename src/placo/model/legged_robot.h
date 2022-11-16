#pragma once

#include "placo/model/mobile_robot.h"

namespace placo
{
class LeggedRobot : public MobileRobot
{
public:
  LeggedRobot(std::string model_directory = "robot/");
};
}  // namespace placo