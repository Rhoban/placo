#include "placo/kinematics/position_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
PositionTask::PositionTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world)
  : frame_index(frame_index), target_world(target_world)
{
}

void PositionTask::support()
{
  columns = solver->robot.frame_support(frame_index);
}

void PositionTask::fill()
{
  pinocchio::SE3 T_world_frame = solver->robot.data->oMf[frame_index];
  mask.R_local_world = T_world_frame.rotation().transpose();
  Eigen::Vector3d error = target_world - T_world_frame.translation();
  solver->robot.compact_frame_jacobian(frame_index, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_a);

  A.resize(mask.rows(), columns.size());
  b.resize(mask.rows(), 1);
  mask.apply(J_a.topRows(3), A);
  mask.apply(error, b);
}

std::string PositionTask::type_name()
{
  return "position";
}

std::string PositionTask::error_unit()
{
  return "m";
}
}  // namespace placo::kinematics