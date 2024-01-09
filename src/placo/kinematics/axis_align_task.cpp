#include "placo/kinematics/axis_align_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
AxisAlignTask::AxisAlignTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d axis_frame,
                             Eigen::Vector3d targetAxis_world)
  : frame_index(frame_index), axis_frame(axis_frame), targetAxis_world(targetAxis_world)
{
}

void AxisAlignTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  auto targetAxis_world_normalized = targetAxis_world.normalized();

  // Here, we will define an "axis frame", which x axis is aligned with the current axis, the z axis is the axis
  // we need to rotate about to correct the error, and y the last axis
  // Thus, expressing the Jacobian in this frame, we can let the rotation about the x axis free, and control
  // the rotation about z to be the error, and about y to be zero.
  Eigen::Matrix3d R_world_axisframe;
  R_world_axisframe.col(0) = (T_world_frame.rotation() * axis_frame).normalized();
  R_world_axisframe.col(2) = R_world_axisframe.col(0).cross(targetAxis_world_normalized).normalized();
  R_world_axisframe.col(1) = R_world_axisframe.col(2).cross(R_world_axisframe.col(0));

  // Computing the error angle we want to compensate
  double error_angle = tools::safe_acos(R_world_axisframe.col(0).dot(targetAxis_world_normalized));

  // We express the Jacobian in the axisframe
  Eigen::MatrixXd J_axisframe = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD).block(3, 0, 3, solver->N);
  J_axisframe = (R_world_axisframe.inverse() * J_axisframe);

  // We only keep y and z in the constraint, since we don't care about rotations about x axis in the axis frame
  A = J_axisframe.block(1, 0, 2, solver->N);
  b = Eigen::Vector2d(0., error_angle);
}

std::string AxisAlignTask::type_name()
{
  return "axis_align";
}

std::string AxisAlignTask::error_unit()
{
  return "rad";
}
}  // namespace placo::kinematics