#include "placo/kinematics/yaw_constraint.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/problem/polygon_constraint.h"

namespace placo::kinematics
{
YawConstraint::YawConstraint(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b,
                             double angle_max)
  : frame_a(frame_a), frame_b(frame_b), angle_max(angle_max)
{
}

void YawConstraint::add_constraint(placo::problem::Problem& problem)
{
  Eigen::Affine3d T_a_b = solver->robot.get_T_a_b(frame_a, frame_b);

  // Jacobian of the rotational velocity expressed in a
  Eigen::MatrixXd J_relative =
      T_a_b.linear() *
          solver->robot.frame_jacobian(frame_b, pinocchio::ReferenceFrame::LOCAL).block(3, 0, 3, solver->N) -
      solver->robot.frame_jacobian(frame_a, pinocchio::ReferenceFrame::LOCAL).block(3, 0, 3, solver->N);
  ;

  // B's x axis in A
  Eigen::Vector3d x_axis = T_a_b.linear().block(0, 0, 3, 1);

  // Current angle
  double alpha = atan2(x_axis.y(), x_axis.x());

  Eigen::Vector3d perp_axis = Eigen::Vector3d::UnitZ().cross(x_axis).normalized();
  Eigen::Vector3d yaw_axis = x_axis.cross(perp_axis).normalized();

  Eigen::MatrixXd J_angle = yaw_axis.transpose() * J_relative;

  // Expression for the angle
  problem::Expression e;
  e.A.resize(2, solver->N);
  e.b.resize(2);
  // First row is angle + J_angle*qd
  e.A.block(0, 0, 1, solver->N) = J_angle;
  e.b(0, 0) = alpha;
  // Second row is -(angle -J_angle*qd)
  e.A.block(1, 0, 1, solver->N) = -J_angle;
  e.b(1, 0) = -alpha;

  problem.add_constraint(e <= angle_max)
      .configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                           problem::ProblemConstraint::Soft,
                 weight);
}

}  // namespace placo::kinematics