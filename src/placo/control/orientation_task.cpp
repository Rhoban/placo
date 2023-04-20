#include "placo/control/orientation_task.h"
#include "placo/control/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo
{
OrientationTask::OrientationTask(RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
  : frame_index(frame_index), R_world_frame(R_world_frame)
{
}

void OrientationTask::update()
{
  auto T_world_frame = solver->robot->get_T_world_frame(frame_index);
  Eigen::Matrix3d M = (R_world_frame * T_world_frame.linear().inverse()).matrix();

  // (R_frame R_current^{-1}) R_current = R_frame
  // |-----------------------|
  //            | This part is the world error that "correct" the rotation
  //            matrix to the desired one
  Eigen::Vector3d error = pinocchio::log3(M);

  Eigen::MatrixXd J = solver->robot->frame_jacobian(frame_index, pinocchio::WORLD);
  Eigen::Matrix3d Jlog;
  pinocchio::Jlog3(M, Jlog);

  A = Jlog * J.block(3, 0, 3, solver->N);
  b = error;
}

std::string OrientationTask::type_name()
{
  return "orientation";
}

std::string OrientationTask::error_unit()
{
  return "rad";
}
}  // namespace placo