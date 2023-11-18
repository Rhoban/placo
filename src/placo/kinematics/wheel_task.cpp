#include "placo/kinematics/wheel_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
WheelTask::WheelTask(std::string joint, double radius, bool omniwheel)
  : joint(joint), radius(radius), omniwheel(omniwheel)
{
  T_world_surface = Eigen::Affine3d::Identity();
}

void WheelTask::update()
{
  Eigen::Affine3d T_world_wheel = solver->robot.get_T_world_frame(joint);
  Eigen::Affine3d T_surface_wheel = T_world_surface.inverse() * T_world_wheel;

  // The "downAxis" is an axis pointing toward the surface in the wheel plane
  Eigen::Vector3d downAxis_wheel = -T_surface_wheel.linear().row(2);
  downAxis_wheel.z() = 0;
  downAxis_wheel.normalize();
  Eigen::Vector3d downAxis_surface = T_surface_wheel.linear() * downAxis_wheel;

  // Building a frame that is coincident with the contact point, x axis in the
  // direction of the wheel, and y axis to the lateral
  Eigen::Affine3d T_surface_contact = Eigen::Affine3d::Identity();
  T_surface_contact.linear().col(0) = T_surface_wheel.linear().col(2).cross(Eigen::Vector3d::UnitZ());
  T_surface_contact.linear().col(0).normalize();
  T_surface_contact.linear().col(1) = T_surface_contact.linear().col(2).cross(T_surface_contact.linear().col(0));
  T_surface_contact.translation() = T_surface_wheel.translation() + radius * downAxis_surface;

  Eigen::Affine3d T_contact_wheel = T_surface_contact.inverse() * T_surface_wheel;

  // Computing contact jacobian
  A = (pinocchio::SE3(T_contact_wheel.matrix()).toActionMatrix() * solver->robot.joint_jacobian(joint, "local"))
          .topRows(3);
  b = Eigen::Vector3d::Zero();
  b(2, 0) = -T_surface_contact.translation().z();

  // With an omniwheel, we remove the lateral sliding constraint (along contact y axis)
  if (omniwheel)
  {
    Eigen::MatrixXd new_A = A({ 0, 2 }, Eigen::all);
    Eigen::MatrixXd new_b = b({ 0, 2 }, Eigen::all);
    A = new_A;
    b = new_b;
  }
}

std::string WheelTask::type_name()
{
  return "wheel";
}

std::string WheelTask::error_unit()
{
  return "m";
}
}  // namespace placo::kinematics