#include "placo/control/relative_position_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
RelativePositionTask::RelativePositionTask(RobotWrapper::FrameIndex frame_a, RobotWrapper::FrameIndex frame_b,
                                           Eigen::Vector3d target)
  : frame_a(frame_a), frame_b(frame_b), target(target)
{
}

void RelativePositionTask::update()
{
  auto T_world_a = solver->robot->get_T_world_frame(frame_a);
  auto T_world_b = solver->robot->get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  // Express the target in the world
  Eigen::Vector3d error_world = T_world_a.linear() * (target - T_a_b.translation());

  // Compute the jacobian locally aligned with the world
  Eigen::MatrixXd J_a =
      solver->robot->frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED).block(0, 0, 3, solver->N);
  Eigen::MatrixXd J_b =
      solver->robot->frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED).block(0, 0, 3, solver->N);

  A = J_b - J_a;
  b = error_world;
}

std::string RelativePositionTask::type_name()
{
  return "relative_position";
}

std::string RelativePositionTask::error_unit()
{
  return "m";
}

}  // namespace placo