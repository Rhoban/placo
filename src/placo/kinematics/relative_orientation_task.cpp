#include "placo/kinematics/relative_orientation_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
RelativeOrientationTask::RelativeOrientationTask(model::RobotWrapper::FrameIndex frame_a,
                                                 model::RobotWrapper::FrameIndex frame_b, Eigen::Matrix3d R_a_b)
  : frame_a(frame_a), frame_b(frame_b), R_a_b(R_a_b)
{
}

void RelativeOrientationTask::update()
{
  Eigen::Affine3d T_world_a = solver->robot.get_T_world_frame(frame_a);
  Eigen::Affine3d T_world_b = solver->robot.get_T_world_frame(frame_b);
  Eigen::Affine3d T_a_b = T_world_a.inverse() * T_world_b;

  Eigen::Vector3d error = pinocchio::log3(R_a_b * T_a_b.linear().transpose());

  Eigen::MatrixXd J_a = solver->robot.frame_jacobian(frame_a, pinocchio::WORLD);
  Eigen::MatrixXd J_b = solver->robot.frame_jacobian(frame_b, pinocchio::WORLD);
  Eigen::MatrixXd J_ab = T_world_a.linear().transpose() * (J_b - J_a).block(3, 0, 3, solver->N);
  Eigen::Matrix3d Jlog;
  pinocchio::Jlog3(R_a_b * T_a_b.linear().transpose(), Jlog);

  A = mask.apply(J_ab);
  b = mask.apply(error);
}

std::string RelativeOrientationTask::type_name()
{
  return "relative_orientation";
}

std::string RelativeOrientationTask::error_unit()
{
  return "rad";
}
}  // namespace placo::kinematics