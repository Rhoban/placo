#include "placo/dynamics/avoid_self_collisions_constraint.h"
#include "placo/dynamics/dynamics_solver.h"

namespace placo::dynamics
{
void AvoidSelfCollisionsConstraint::add_constraint(problem::Problem& problem, problem::Expression& tau)
{
  if (solver->dt == 0.)
  {
    throw std::runtime_error("AvoidSelfCollisionsConstraint::add_constraint: dt is not set");
  }

  std::vector<model::RobotWrapper::Distance> distances = solver->robot.distances();

  // The constraint depends on the union of the supports of the joints involved in close pairs
  int constraints = 0;
  columns.clear();
  for (auto& distance : distances)
  {
    if (distance.min_distance < self_collisions_trigger)
    {
      constraints += 1;
      model::RobotWrapper::merge_supports(columns, solver->robot.joint_support(distance.parentA), columns_buffer);
      model::RobotWrapper::merge_supports(columns_buffer, solver->robot.joint_support(distance.parentB), columns);
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

  Eigen::VectorXd qd_columns(columns.size());
  for (int k = 0; k < (int)columns.size(); k++)
  {
    qd_columns[k] = solver->robot.state.qd[columns[k]];
  }

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

      // Jacobians of the witness points (world axes), and their time variations (the world ones, shifted to the
      // points)
      pinocchio::SE3 T_world_A(Eigen::Matrix3d::Identity(), distance.pointA);
      pinocchio::SE3 T_world_B(Eigen::Matrix3d::Identity(), distance.pointB);
      solver->robot.compact_jacobian(distance.parentA, T_world_A, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_a);
      solver->robot.compact_jacobian(distance.parentB, T_world_B, pinocchio::LOCAL_WORLD_ALIGNED, columns, J_b);
      solver->robot.compact_jacobian_time_variation(distance.parentA, T_world_A, pinocchio::WORLD, columns, dJ_a);
      solver->robot.compact_jacobian_time_variation(distance.parentB, T_world_B, pinocchio::WORLD, columns, dJ_b);
      for (int k = 0; k < (int)columns.size(); k++)
      {
        dJ_a.col(k).head<3>() += dJ_a.col(k).tail<3>().cross(distance.pointA);
        dJ_b.col(k).head<3>() += dJ_b.col(k).tail<3>().cross(distance.pointB);
      }

      // We want: current_distance + J dq >= margin
      Eigen::RowVectorXd J = n.transpose() * (J_b.topRows(3) - J_a.topRows(3));
      double dJ = n.transpose() * (dJ_b.topRows(3) - dJ_a.topRows(3)) * qd_columns;

      // Computing xdd_safe from qdd_safe
      double xdd_safe = 0.0;
      for (int k = 0; k < (int)columns.size(); k++)
      {
        if (columns[k] >= 6)
        {
          xdd_safe += fabs(J[k]) * solver->qdd_safe[columns[k]];
        }
      }
      xdd_safe = 0.5 * xdd_safe;

      if (distance.min_distance >= self_collisions_margin)
      {
        // We prevent excessive velocity towards the collision
        double error = distance.min_distance - self_collisions_margin;
        double xd = J.dot(qd_columns);
        double xd_max = sqrt(2. * error * xdd_safe);

        constraint.expression.A.row(row) = solver->dt * J;
        constraint.expression.b[row] = solver->dt * dJ + xd + xd_max;
      }
      else
      {
        // We push outward the collision
        constraint.expression.A.row(row) = J;
        constraint.expression.b[row] = -xdd_safe;
      }

      row += 1;
    }
  }

  constraint.configure(
      priority == Priority::Soft ? problem::ProblemConstraint::Soft : problem::ProblemConstraint::Hard, weight);
}
};  // namespace placo::dynamics