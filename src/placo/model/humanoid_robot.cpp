#include "placo/model/humanoid_robot.h"
#include "placo/utils.h"

namespace placo
{
HumanoidRobot::Side HumanoidRobot::string_to_side(const std::string& str)
{
  return (str == "right") ? Right : (str == "left") ? Left : Both;
}

HumanoidRobot::Side HumanoidRobot::other_side(Side side)
{
  return (side == Left) ? Right : Left;
}

HumanoidRobot::HumanoidRobot(std::string model_directory) : RobotWrapper(model_directory)
{
}

void HumanoidRobot::load()
{
  this->RobotWrapper::load();

  support_side = Both;
  T_world_support.setIdentity();

  left_foot = get_frame_index("left_foot");
  right_foot = get_frame_index("right_foot");
  trunk = get_frame_index("trunk");

  ensure_on_floor();
}

Eigen::Affine3d HumanoidRobot::get_T_world_left()
{
  return get_T_world_frame(left_foot);
}

Eigen::Affine3d HumanoidRobot::get_T_world_right()
{
  return get_T_world_frame(right_foot);
}

Eigen::Affine3d HumanoidRobot::get_T_world_trunk()
{
  return get_T_world_frame(trunk);
}

void HumanoidRobot::update_support_side(HumanoidRobot::Side new_side)
{
  if (new_side != support_side)
  {
    if (support_side != Both)
    {
      flying_side = other_side(flying_side);
    }

    // Updating the support frame to this frame
    support_side = new_side;

    if (support_side != Both)
    {
      update_kinematics();

      // Retrieving the current support configuration
      auto T_world_newSupport = get_T_world_frame(flying_frame());

      // Projecting it on the floor
      T_world_support = flatten_on_floor(T_world_newSupport);

      // GOOD PRACTICE BUT CAUSE A JUMP
      // ensure_on_floor();
    }
  }
}

void HumanoidRobot::ensure_on_floor()
{
  // Updating the floating base so that the foot is where we want
  update_kinematics();
  set_T_world_frame(support_frame(), T_world_support);
  update_kinematics();
}

RobotWrapper::FrameIndex HumanoidRobot::support_frame()
{
  return flying_side == Left ? right_foot : left_foot;
}

RobotWrapper::FrameIndex HumanoidRobot::flying_frame()
{
  return flying_side == Left ? left_foot : right_foot;
}

void HumanoidRobot::update_support_side(const std::string& side)
{
  update_support_side(string_to_side(side));
}
}  // namespace placo