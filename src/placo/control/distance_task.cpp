#include "placo/control/distance_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
DistanceTask::DistanceTask(MobileRobot::FrameIndex frame_a, MobileRobot::FrameIndex frame_b, double distance)
  : frame_a(frame_a), frame_b(frame_b), distance(distance)
{
  b = Eigen::MatrixXd(1, 1);
}

void DistanceTask::update()
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);

  Eigen::Vector3d ab_world = T_world_b.translation() - T_world_a.translation();

  double error = distance - ab_world.norm();
  Eigen::Vector3d direction = ab_world.normalized();

  auto J_a = solver->robot.frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED);
  auto J_b = solver->robot.frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED);
  A = direction.transpose() * (J_b - J_a).block(0, 0, 3, solver->N);
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
}  // namespace placo