#include "placo/kinematics/avoid_self_collisions_constraint.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void AvoidSelfCollisionsConstraint::add_constraint(placo::problem::Problem& problem)
{
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

      Eigen::MatrixXd X_B_world = pinocchio::SE3(Eigen::Matrix3d::Identity(), -distance.pointB).toActionMatrix();
      Eigen::MatrixXd JB = X_B_world * solver->robot.joint_jacobian(distance.parentB, pinocchio::ReferenceFrame::WORLD);

      // We want: current_distance + J dq >= margin
      e.A.block(constraint, 0, 1, solver->N) = n.transpose() * (JB - JA).block(0, 0, 3, solver->N);
      e.b[constraint] = distance.min_distance - self_collisions_margin;

      constraint += 1;
    }
  }

  problem.add_constraint(e >= 0).configure(
      priority == Priority::Soft ? problem::ProblemConstraint::Soft : problem::ProblemConstraint::Hard, weight);
}
};  // namespace placo::kinematics