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

  Eigen::MatrixXd J_a = solver->robot.frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::MatrixXd J_b = solver->robot.frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::MatrixXd J_distance = direction.transpose() * (J_b - J_a).block(0, 0, 3, solver->N);

  // Expression for the angle
  problem::Expression e;
  e.A.resize(1, solver->N);
  e.b.resize(1);

  e.A.row(0) = J_distance;
  e.b(0) = distance;

  problem.add_constraint(e <= distance_max)
      .configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                           problem::ProblemConstraint::Soft,
                 weight);
}

}  // namespace placo::kinematics