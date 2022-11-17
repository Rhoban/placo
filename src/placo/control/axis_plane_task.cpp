#include "placo/control/axis_plane_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
AxisPlaneTask::AxisPlaneTask(RobotWrapper::FrameIndex frame_index, Eigen::Vector3d axis_frame,
                             Eigen::Vector3d normal_world)
  : frame_index(frame_index), axis_frame(axis_frame), normal_world(normal_world)
{
  b = Eigen::MatrixXd(1, 1);
}

void AxisPlaneTask::update()
{
  auto T_world_frame = solver->robot->get_T_world_frame(frame_index);

  Eigen::Vector3d axis_world = (T_world_frame.rotation() * axis_frame).normalized();
  Eigen::Vector3d normal_world_normalized = normal_world.normalized();
  Eigen::Vector3d rotate_axis = axis_world.cross(normal_world_normalized).normalized();
  Eigen::Vector3d axis_target = normal_world_normalized.cross(rotate_axis).normalized();
  double error = safe_acos(axis_world.dot(normal_world_normalized)) - (M_PI / 2);

  Eigen::MatrixXd J = solver->robot->frame_jacobian(frame_index, pinocchio::WORLD);
  A = rotate_axis.transpose() * J.block(3, 0, 3, solver->N);
  b(0, 0) = error;
}

std::string AxisPlaneTask::type_name()
{
  return "axis_plane";
}

std::string AxisPlaneTask::error_unit()
{
  return "rad";
}
}  // namespace placo