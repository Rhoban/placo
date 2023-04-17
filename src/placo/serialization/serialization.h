#pragma once

#include "placo/model/robot_wrapper.h"
#include "placo/model/humanoid_robot.h"
#include <Eigen/Dense>
#include "placo.pb.h"

namespace placo
{
// Position
void serialize_position(placo_proto::Position* proto_position, Eigen::Vector3d position);
void serialize_position(placo_proto::Position* proto_position, Eigen::Affine3d T);
Eigen::Vector3d unserialize_position(placo_proto::Position* proto_position);

// Orientation
void serialize_orientation(placo_proto::Orientation* proto_orientation, Eigen::Matrix3d orientation);
void serialize_orientation(placo_proto::Orientation* proto_orientation, Eigen::Affine3d T);
Eigen::Matrix3d unserialize_orientation(placo_proto::Orientation* proto_orientation);

// Pose
void serialize_pose(placo_proto::Pose* proto_T, Eigen::Affine3d T);
Eigen::Affine3d unserialize_pose(placo_proto::Pose* proto_T);

// Robot state
void serialize_robot_state(placo_proto::RobotState* proto_state, RobotWrapper& robot);
void unserialize_robot_state(placo_proto::RobotState* proto_state, RobotWrapper& robot);

// Humanoid robot state
void serialize_humanoid_robot_state(placo_proto::HumanoidRobotState* proto_state, HumanoidRobot& robot);
void unserialize_humanoid_robot_state(placo_proto::HumanoidRobotState* proto_state, HumanoidRobot& robot);
};  // namespace placo