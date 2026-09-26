#include "placo/kinematics/cone_constraint.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/problem/polygon_constraint.h"

namespace placo::kinematics
{
ConeConstraint::ConeConstraint(model::RobotWrapper::FrameIndex frame_a, model::RobotWrapper::FrameIndex frame_b,
                               double angle_max)
  : frame_a(frame_a), frame_b(frame_b), angle_max(angle_max)
{
}

void ConeConstraint::add_constraint(placo::problem::Problem& problem)
{
  Eigen::Affine3d T_a_b = solver->robot.get_T_a_b(frame_a, frame_b);

  // Axis expressed in the cone frame
  Eigen::Vector3d axis_cone = T_a_b.rotation() * Eigen::Vector3d::UnitZ();

  // Jacobian of the rotational velocity expressed in a
  model::RobotWrapper::merge_supports(solver->robot.frame_support(frame_a), solver->robot.frame_support(frame_b),
                                      columns);
  solver->robot.compact_frame_jacobian(frame_a, pinocchio::LOCAL, columns, J_a);
  solver->robot.compact_frame_jacobian(frame_b, pinocchio::LOCAL, columns, J_b);
  Eigen::Matrix<double, 3, Eigen::Dynamic> J_cone = T_a_b.linear() * J_b.bottomRows(3) - J_a.bottomRows(3);

  // Preparing the constraint: alpha + J_slice dq <= angle_max for each slice
  problem::ProblemConstraint& constraint = problem.add_constraint();
  constraint.type = problem::ProblemConstraint::Inequality;
  constraint.columns = columns;
  constraint.expression.A.resize(N, columns.size());
  constraint.expression.b.resize(N);

  // Current axis orientation
  double slice_alpha_offset = atan2(axis_cone.y(), axis_cone.x());

  for (int k = 0; k < N; k++)
  {
    // Building the slice frame
    double slice_alpha = slice_alpha_offset + (k * 2 * range / N) - range;
    Eigen::Matrix3d R_cone_slice = Eigen::AngleAxisd(slice_alpha, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Axis angle in the slice frame
    Eigen::Vector3d axis_slice = R_cone_slice.transpose() * axis_cone;
    double alpha = atan2(axis_slice.x(), axis_slice.z());

    // Jacobian of the angle in the slice frame
    Eigen::Vector3d rotation_axis = R_cone_slice.col(1);
    constraint.expression.A.row(k).noalias() = -rotation_axis.transpose() * J_cone;
    constraint.expression.b(k) = angle_max - alpha;
  }

  constraint.configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard : problem::ProblemConstraint::Soft, weight);
}

}  // namespace placo::kinematics