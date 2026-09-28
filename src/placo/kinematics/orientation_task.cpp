#include "placo/kinematics/orientation_task.h"
#include "placo/kinematics/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo::kinematics
{
OrientationTask::OrientationTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
  : frame_index(frame_index), R_world_frame(R_world_frame)
{
}

void OrientationTask::support()
{
  columns = solver->robot.frame_support(frame_index);
}

void OrientationTask::fill()
{
  pinocchio::SE3 T_world_frame = solver->robot.data->oMf[frame_index];
  Eigen::Matrix3d M = R_world_frame * T_world_frame.rotation().transpose();
  Eigen::Vector3d error = pinocchio::log3(M);
  solver->robot.compact_frame_jacobian(frame_index, pinocchio::WORLD, columns, J_a);

  mask.R_local_world = R_world_frame.transpose();
  A.resize(mask.rows(), columns.size());
  b.resize(mask.rows(), 1);
  mask.apply(J_a.bottomRows(3), A);
  mask.apply(error, b);
}

std::string OrientationTask::type_name()
{
  return "orientation";
}

std::string OrientationTask::error_unit()
{
  return "rad";
}
}  // namespace placo::kinematics