#include "placo/model/humanoid_robot.h"
#include "placo/utils.h"
#include "pinocchio/math/rpy.hpp"

namespace placo
{
HumanoidRobot::HumanoidRobot(std::string model_directory) : RobotWrapper(model_directory)
{
  initialize();
  ensure_on_floor();
}

void HumanoidRobot::initialize()
{
  this->RobotWrapper::load();

  support_side = Both;
  T_world_support.setIdentity();

  left_foot = get_frame_index("left_foot");
  right_foot = get_frame_index("right_foot");
  trunk = get_frame_index("trunk");

  ensure_on_floor();
}

HumanoidRobot::Side HumanoidRobot::string_to_side(const std::string& str)
{
  return (str == "right") ? Right : (str == "left") ? Left : Both;
}

HumanoidRobot::Side HumanoidRobot::other_side(Side side)
{
  return (side == Left) ? Right : Left;
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
      auto T_world_newSupport = get_T_world_frame(support_frame());

      // Projecting it on the floor
      T_world_support = flatten_on_floor(T_world_newSupport);

      ensure_on_floor();
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

void HumanoidRobot::readFromHistories(rhoban_utils::HistoryCollection& histories, double timestamp, bool use_imu)
{
  // Updating DOFs from replay
  for (const std::string& name : joint_names())
  {
    set_joint(name, histories.number("read:" + name)->interpolate(timestamp));
  }

  // Set the support foot on the floor
  if (!use_imu)
  {
    double left_pressure = histories.number("left_pressure_weight")->interpolate(timestamp);
    double right_pressure = histories.number("right_pressure_weight")->interpolate(timestamp);
    if (left_pressure > right_pressure)
    {
      update_support_side(Left);
    }
    else
    {
      update_support_side(Right);
    }

    ensure_on_floor();
    update_kinematics();
  }

  // Setting the trunk orientation from the IMU
  else
  {
    double imuYaw = histories.angle("imu_gyro_yaw")->interpolate(timestamp);
    double imuPitch = histories.angle("imu_pitch")->interpolate(timestamp);
    double imuRoll = histories.angle("imu_roll")->interpolate(timestamp);

    Eigen::Affine3d T_world_trunk = get_T_world_trunk();
    T_world_trunk.linear() = pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(imuRoll, imuPitch, imuYaw));

    update_kinematics();
    set_T_world_frame(trunk, T_world_trunk);
    update_kinematics();
  }
}
}  // namespace placo