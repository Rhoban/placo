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

  Eigen::MatrixXd R_world_a = T_world_a.linear();

  // Express the error
  Eigen::Vector3d error = target - T_a_b.translation();

  Eigen::MatrixXd J_a_pos =
      solver->robot->frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED).block(0, 0, 3, solver->N);
  Eigen::MatrixXd J_a_rot =
      solver->robot->frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED).block(3, 0, 3, solver->N);
  Eigen::MatrixXd J_b_pos =
      solver->robot->frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED).block(0, 0, 3, solver->N);

  A = R_world_a.transpose() * (J_b_pos - J_a_pos) +
      pinocchio::skew(T_a_b.translation()) * R_world_a.transpose() * J_a_rot;
  b = error;
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