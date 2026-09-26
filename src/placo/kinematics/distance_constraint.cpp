#include "placo/kinematics/distance_constraint.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/problem/polygon_constraint.h"

namespace placo::kinematics
{
DistanceConstraint::DistanceConstraint(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b,
                                       double distance_max)
  : frame_a(frame_a), frame_b(frame_b), distance_max(distance_max)
{
}

void DistanceConstraint::add_constraint(placo::problem::Problem& problem)
{
  auto T_world_a = solver->robot.get_T_world_frame(frame_a);
  auto T_world_b = solver->robot.get_T_world_frame(frame_b);

  Eigen::Vector3d ab_world = T_world_b.translation() - T_world_a.translation();

  double distance = ab_world.norm();
  Eigen::Vector3d direction = ab_world.normalized();

  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a), solver->robot.frame_support(frame_b),
                                      columns);
  solver->robot.compact_frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_a);
  solver->robot.compact_frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_b);

  // distance + J_distance dq <= distance_max
  problem::ProblemConstraint& constraint = problem.add_constraint();
  constraint.type = problem::ProblemConstraint::Inequality;
  constraint.columns = columns;
  constraint.expression.A.noalias() = -direction.transpose() * (J_b.topRows(3) - J_a.topRows(3));
  constraint.expression.b.setConstant(1, distance_max - distance);
  constraint.configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard : problem::ProblemConstraint::Soft, weight);
}

}  // namespace placo::kinematics