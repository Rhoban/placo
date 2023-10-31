#include "placo/kinematics/orientation_task.h"
#include "placo/kinematics/kinematics_solver.h"
#include <pinocchio/spatial/explog.hpp>

namespace placo::kinematics
{
OrientationTask::OrientationTask(RobotWrapper::FrameIndex frame_index, Eigen::Matrix3d R_world_frame)
  : frame_index(frame_index), R_world_frame(R_world_frame)
{
}

void OrientationTask::update()
{
  auto T_world_frame = solver->robot.get_T_world_frame(frame_index);
  Eigen::MatrixXd J, Jlog, error;

  if (mask.local)
  {
    // When expressing the error in local:
    // R exp(w) = Rd
    // Rd.T R exp(w) = I
    // log3(Rd.T R exp(w)) = 0
    // Jlog(Rd.T R) w = -log3(Rd.T R)

    Eigen::Matrix3d M = (R_world_frame.transpose() * T_world_frame.linear()).matrix();
    error = -pinocchio::log3(M);
    J = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL);

    pinocchio::Jlog3(M, Jlog);
  }
  else
  {
    // When expressing the error in the world:
    // exp(w) R = Rd
    // exp(w) R Rd.T = I
    // log3(exp(w) R Rd.T) = 0
    // log3(Rd R.T exp(-w)) = 0
    // log3(Rd R.T) - Jlog(Rd R.T) w = 0
    // Jlog(Rd R.T) w = log3(Rd R.T)

    Eigen::Matrix3d M = (R_world_frame * T_world_frame.linear().transpose()).matrix();
    error = pinocchio::log3(M);
    J = solver->robot.frame_jacobian(frame_index, pinocchio::WORLD);
    pinocchio::Jlog3(M, Jlog);
  }

  A = mask.apply(Jlog * J.block(3, 0, 3, solver->N));
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