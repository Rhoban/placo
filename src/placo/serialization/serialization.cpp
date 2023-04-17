#include "placo/serialization/serialization.h"

namespace placo
{
void serialize_position(placo_proto::Position* proto_position, Eigen::Vector3d position)
{
  proto_position->set_x(position.x());
  proto_position->set_y(position.y());
  proto_position->set_z(position.z());
}

void serialize_position(placo_proto::Position* proto_position, Eigen::Affine3d T)
{
  serialize_position(proto_position, Eigen::Vector3d(T.translation()));
}

Eigen::Vector3d unserialize_position(placo_proto::Position* proto_position)
{
  return Eigen::Vector3d(proto_position->x(), proto_position->y(), proto_position->z());
}

void serialize_orientation(placo_proto::Orientation* proto_orientation, Eigen::Matrix3d orientation)
{
  auto rotation = Eigen::Quaterniond(orientation);
  proto_orientation->set_qw(rotation.w());
  proto_orientation->set_qx(rotation.x());
  proto_orientation->set_qy(rotation.y());
  proto_orientation->set_qz(rotation.z());
}

void serialize_orientation(placo_proto::Orientation* proto_orientation, Eigen::Affine3d T)
{
  serialize_orientation(proto_orientation, T.linear());
}

Eigen::Matrix3d unserialize_orientation(placo_proto::Orientation* proto_orientation)
{
  auto rotation = Eigen::Quaterniond(proto_orientation->qw(), proto_orientation->qx(), proto_orientation->qy(),
                                     proto_orientation->qz());

  return rotation.matrix();
}

void serialize_pose(placo_proto::Pose* proto_T, Eigen::Affine3d T)
{
  serialize_position(proto_T->mutable_position(), T);
  serialize_orientation(proto_T->mutable_orientation(), T);
}

Eigen::Affine3d unserialize_pose(placo_proto::Pose* proto_T)
{
  Eigen::Affine3d T;
  T.translation() = unserialize_position(proto_T->mutable_position());
  T.linear() = unserialize_orientation(proto_T->mutable_orientation());

  return T;
}

void serialize_robot_state(placo_proto::RobotState* proto_state, RobotWrapper& robot)
{
  for (auto& q : robot.state.q)
  {
    proto_state->add_q(q);
  }
  for (auto& dq : robot.state.qd)
  {
    proto_state->add_dq(dq);
  }
}

void unserialize_robot_state(placo_proto::RobotState* proto_state, RobotWrapper& robot)
{
  for (int k = 0; k < proto_state->mutable_q()->size(); k++)
  {
    robot.state.q[k] = proto_state->mutable_q()->at(k);
  }
  for (int k = 0; k < proto_state->mutable_dq()->size(); k++)
  {
    robot.state.qd[k] = proto_state->mutable_dq()->at(k);
  }
}

void serialize_humanoid_robot_state(placo_proto::HumanoidRobotState* proto_state, HumanoidRobot& robot)
{
  serialize_robot_state(proto_state->mutable_robotstate(), robot);
  proto_state->set_support_side((placo_proto::HumanoidRobotState_Side)robot.support_side);
  serialize_pose(proto_state->mutable_t_world_support(), robot.T_world_support);
}

void unserialize_humanoid_robot_state(placo_proto::HumanoidRobotState* proto_state, HumanoidRobot& robot)
{
  unserialize_robot_state(proto_state->mutable_robotstate(), robot);
  robot.support_side = (HumanoidRobot::Side)proto_state->support_side();
  robot.flying_side = robot.other_side(robot.support_side);
  robot.T_world_support = unserialize_pose(proto_state->mutable_t_world_support());
}

}  // namespace placo