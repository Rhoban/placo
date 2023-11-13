#include "placo/kinematics/position_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
PositionTask::PositionTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Vector3d target_world)
  : frame_index(frame_index), target_world(target_world)
{
}

void PositionTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  mask.R_local_world = T_world_frame.linear().transpose();
  Eigen::Vector3d error = target_world - T_world_frame.translation();
  Eigen::MatrixXd J = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL_WORLD_ALIGNED);

  A = mask.apply(J.block(0, 0, 3, solver->N));
  b = mask.apply(error);
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