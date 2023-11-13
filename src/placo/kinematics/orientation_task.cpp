#include "placo/kinematics/orientation_task.h"
#include "placo/kinematics/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo::kinematics
{
OrientationTask::OrientationTask(model::RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
  : frame_index(frame_index), R_world_frame(R_world_frame)
{
}

void OrientationTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  Eigen::MatrixXd J, error;

  Eigen::Matrix3d M = (R_world_frame * T_world_frame.linear().transpose()).matrix();
  error = pinocchio::log3(M);
  J = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD);

  mask.R_local_world = R_world_frame.transpose();
  A = mask.apply(J.block(3, 0, 3, solver->N));
  b = mask.apply(error);
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