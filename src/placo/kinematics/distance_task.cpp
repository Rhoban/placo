#include "placo/kinematics/distance_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
DistanceTask::DistanceTask(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b,
                           double distance)
  : frame_a(frame_a), frame_b(frame_b), distance(distance)
{
  b = Eigen::MatrixXd(1, 1);
}

void DistanceTask::support()
{
  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a), solver->robot.frame_support(frame_b),
                                      columns);
}

void DistanceTask::fill()
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);

  Eigen::Vector3d ab_world = T_world_b.translation() - T_world_a.translation();

  double error = distance - ab_world.norm();
  Eigen::Vector3d direction = ab_world.normalized();

  solver->robot.compact_frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_a);
  solver->robot.compact_frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_b);
  A.noalias() = direction.transpose() * (J_b.topRows(3) - J_a.topRows(3));
  b(0, 0) = error;
}

std::string DistanceTask::type_name()
{
  return "distance";
}

std::string DistanceTask::error_unit()
{
  return "m";
}
}  // namespace placo::kinematics