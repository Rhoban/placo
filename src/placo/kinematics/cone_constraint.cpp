#include "placo/kinematics/cone_constraint.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/problem/polygon_constraint.h"

namespace placo::kinematics
{
ConeConstraint::ConeConstraint(model::RobotWrapper::FrameIndex frame, double alpha_max, Eigen::Affine3d T_world_cone)
  : frame(frame), alpha_max(alpha_max), T_world_cone(T_world_cone)
{
}

void ConeConstraint::add_constraint(placo::problem::Problem& problem)
{
  Eigen::Affine3d T_world_local = solver->robot.get_T_world_frame(frame);
  Eigen::Affine3d T_cone_local = T_world_cone.inverse() * T_world_local;
  Eigen::Matrix3d R_cone_world = T_world_cone.rotation().transpose();

  // Axis expressed in the cone frame
  Eigen::Vector3d axis_cone = T_cone_local.rotation() * Eigen::Vector3d::UnitZ();

  // Jacobian of the body expressed in the cone frame
  Eigen::MatrixXd J_cone =
      R_cone_world * solver->robot.frame_jacobian(frame, pinocchio::ReferenceFrame::WORLD).block(3, 0, 3, solver->N);

  // Preparing the expression
  problem::Expression e;
  e.A.resize(N, solver->N);
  e.b.resize(N);

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
    Eigen::MatrixXd J_slice = rotation_axis.transpose() * J_cone;

    e.A.row(k) = J_slice;
    e.b(k) = alpha;
  }

  problem.add_constraint(e <= alpha_max)
      .configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                           problem::ProblemConstraint::Soft,
                 weight);
}

}  // namespace placo::kinematics