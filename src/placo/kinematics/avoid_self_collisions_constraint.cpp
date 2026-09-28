#include "placo/kinematics/avoid_self_collisions_constraint.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void AvoidSelfCollisionsConstraint::add_constraint(placo::problem::Problem& problem)
{
  std::vector<model::RobotWrapper::Distance> distances = solver->robot.distances();

  // The constraint depends on the union of the supports of the joints involved in close pairs
  int constraints = 0;
  columns.clear();
  for (auto& distance : distances)
  {
    if (distance.min_distance < self_collisions_trigger)
    {
      constraints += 1;
      model::RobotWrapper::merge_supports(columns, solver->robot.joint_support(distance.parentA), J_columns);
      model::RobotWrapper::merge_supports(J_columns, solver->robot.joint_support(distance.parentB), columns);
    }
  }

  if (constraints == 0)
  {
    return;
  }

  problem::ProblemConstraint& constraint = problem.add_constraint();
  constraint.type = problem::ProblemConstraint::Inequality;
  constraint.columns = columns;
  constraint.expression.A.resize(constraints, columns.size());
  constraint.expression.b.resize(constraints);
  int row = 0;

  for (auto& distance : distances)
  {
    if (distance.min_distance < self_collisions_trigger)
    {
      Eigen::Vector3d v = distance.pointB - distance.pointA;
      Eigen::Vector3d n = v.normalized();

      if (distance.min_distance < 0)
      {
        // If the distance is negative, the points "cross" and this vector should point the other way around
        n = -n;
      }

      // Jacobians of the witness points (world axes)
      pinocchio::SE3 T_world_A(Eigen::Matrix3d::Identity(), distance.pointA);
      pinocchio::SE3 T_world_B(Eigen::Matrix3d::Identity(), distance.pointB);
      solver->robot.compact_jacobian(distance.parentA, T_world_A, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_a);
      solver->robot.compact_jacobian(distance.parentB, T_world_B, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_b);

      // We want: current_distance + J dq >= margin
      constraint.expression.A.row(row).noalias() = n.transpose() * (J_b.topRows(3) - J_a.topRows(3));
      constraint.expression.b[row] = distance.min_distance - self_collisions_margin;

      row += 1;
    }
  }

  constraint.configure(
      priority == Priority::Soft ? problem::ProblemConstraint::Soft : problem::ProblemConstraint::Hard, weight);
}
};  // namespace placo::kinematics