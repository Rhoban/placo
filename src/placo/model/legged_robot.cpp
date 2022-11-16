#include "placo/model/legged_robot.h"
#include "placo/utils.h"

namespace placo
{
LeggedRobot::LeggedRobot(std::string model_directory) : MobileRobot(model_directory)
{
  support_frame = -1;
  T_world_support.setIdentity();
}

void LeggedRobot::update_support(MobileRobot::FrameIndex new_support_frame)
{
  if (new_support_frame != support_frame)
  {
    update_kinematics();

    // Retrieving the current support configuration
    auto T_world_newSupport = get_T_world_frame(new_support_frame);

    // Projecting it on the floor
    T_world_support = flatten_on_floor(T_world_newSupport);

    // Updating the support frame to this frame
    support_frame = new_support_frame;

    // Updating the floating base so that the foot is where we want
    set_T_world_frame(support_frame, T_world_support);
  }
}

void LeggedRobot::update_support(const std::string& frame)
{
  update_support(get_frame_index(frame));
}
}  // namespace placo