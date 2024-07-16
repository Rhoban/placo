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

  int constraints = 0;

  for (auto& distance : distances)
  {
    if (distance.min_distance < self_collisions_trigger)
    {
      constraints += 1;
    }
  }

  if (constraints == 0)
  {
    return;
  }

  problem::Expression e;
  e.A = Eigen::MatrixXd(constraints, solver->N);
  e.b = Eigen::VectorXd(constraints);
  int constraint = 0;

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

      Eigen::MatrixXd X_A_world = pinocchio::SE3(Eigen::Matrix3d::Identity(), -distance.pointA).toActionMatrix();
      Eigen::MatrixXd JA = X_A_world * solver->robot.joint_jacobian(distance.parentA, pinocchio::ReferenceFrame::WORLD);
      Eigen::MatrixXd dJA =
          X_A_world * solver->robot.joint_jacobian_time_variation(distance.parentA, pinocchio::ReferenceFrame::WORLD);

      Eigen::MatrixXd X_B_world = pinocchio::SE3(Eigen::Matrix3d::Identity(), -distance.pointB).toActionMatrix();
      Eigen::MatrixXd JB = X_B_world * solver->robot.joint_jacobian(distance.parentB, pinocchio::ReferenceFrame::WORLD);
      Eigen::MatrixXd dJB =
          X_B_world * solver->robot.joint_jacobian_time_variation(distance.parentB, pinocchio::ReferenceFrame::WORLD);

      // We want: current_distance + J dq >= margin
      Eigen::MatrixXd J = n.transpose() * (JB - JA).block(0, 0, 3, solver->N);
      Eigen::VectorXd dJ = n.transpose() * (dJB - dJA).block(0, 0, 3, solver->N) * solver->robot.state.qd;

      // Computing xdd_safe from qdd_safe
      double xdd_safe = 0.0;
      for (int k = 6; k < solver->N; k++)
      {
        xdd_safe += fabs(J(0, k)) * solver->qdd_safe[k];
      }
      xdd_safe = 0.5 * xdd_safe;

      if (distance.min_distance >= self_collisions_margin)
      {
        // We prevent excessive velocity towards the collision
        double error = distance.min_distance - self_collisions_margin;
        double xd = (J * solver->robot.state.qd)(0, 0);
        double xd_max = sqrt(2. * error * xdd_safe);

        e.A.block(constraint, 0, 1, solver->N) = solver->dt * J;
        e.b[constraint] = solver->dt * dJ[0] + xd + xd_max;
      }
      else
      {
        // We push outward the collision
        e.A.block(constraint, 0, 1, solver->N) = J;
        e.b[constraint] = -xdd_safe;
      }

      constraint += 1;
    }
  }

  problem.add_constraint(e >= 0).configure(
      priority == Priority::Soft ? problem::ProblemConstraint::Soft : problem::ProblemConstraint::Hard, weight);
}
};  // namespace placo::dynamics