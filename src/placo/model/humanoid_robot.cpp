#include "placo/model/humanoid_robot.h"
#include "placo/utils.h"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace placo
{
HumanoidRobot::HumanoidRobot(std::string model_directory, int flags, std::string urdf_content)
  : RobotWrapper(model_directory, flags, urdf_content)
{
  initialize();

  // Measuring some distances
  dist_y_trunk_foot = fabs(get_T_a_b("trunk", "left_hip_yaw").translation().y());
  dist_z_pan_tilt = get_T_a_b("head_base", "head_pitch").translation().z();
  dist_z_pan_camera = get_T_a_b("head_base", "camera").translation().z();
}

void HumanoidRobot::initialize()
{
  check_expected();
  init_config();
}

void HumanoidRobot::init_config()
{
  support_side = Left;

  T_world_support.setIdentity();

  left_foot = get_frame_index("left_foot");
  right_foot = get_frame_index("right_foot");
  trunk = get_frame_index("trunk");

  ensure_on_floor();
}

HumanoidRobot::Side HumanoidRobot::string_to_side(const std::string& str)
{
  return (str == "right") ? Right : Left;
}

HumanoidRobot::Side HumanoidRobot::other_side(Side side)
{
  return (side == Left) ? Right : Left;
}

Eigen::Affine3d HumanoidRobot::get_T_world_left()
{
  return get_T_world_frame(left_foot);
}

Eigen::Affine3d HumanoidRobot::get_T_world_right()
{
  return get_T_world_frame(right_foot);
}

Eigen::Affine3d HumanoidRobot::get_T_world_trunk()
{
  return get_T_world_frame(trunk);
}

Eigen::Affine3d HumanoidRobot::get_T_world_self()
{
  return flatten_on_floor(placo::interpolate_frames(get_T_world_right(), get_T_world_left(), 0.5));
}

void HumanoidRobot::update_support_side(HumanoidRobot::Side new_side)
{
  if (new_side != support_side)
  {
    // Updating the support frame to this frame
    support_side = new_side;

    update_kinematics();

    // If we have the 2 feet on the ground, we will use the left one to ensure we are on the floor
    T_world_support = flatten_on_floor(get_T_world_frame(support_frame()));
  }
}

void HumanoidRobot::update_trunk_angular_velocity(double elapsed)
{
  omega_b = pinocchio::log3(R_world_trunk.transpose() * get_T_world_trunk().rotation()) / elapsed;
  R_world_trunk = get_T_world_trunk().rotation();
}

void HumanoidRobot::ensure_on_floor()
{
  // Updating the floating base so that the foot is where we want
  update_kinematics();
  set_T_world_frame(support_frame(), T_world_support);
  update_kinematics();
}

RobotWrapper::FrameIndex HumanoidRobot::support_frame()
{
  return support_side == Left ? left_foot : right_foot;
}

RobotWrapper::FrameIndex HumanoidRobot::flying_frame()
{
  return support_side == Left ? right_foot : left_foot;
}

void HumanoidRobot::update_support_side(const std::string& side)
{
  update_support_side(string_to_side(side));
}

Eigen::Vector3d HumanoidRobot::get_com_velocity(Eigen::VectorXd qd_a, Side support, Eigen::Vector3d omega_b)
{
  // CoM Jacobians
  Eigen::Matrix3Xd J_C = com_jacobian();
  Eigen::Matrix3Xd J_u_C = J_C.leftCols(6);
  Eigen::Matrix3Xd J_a_C = J_C.rightCols(20);

  // Support foot
  Eigen::MatrixXd J_contact = support == placo::HumanoidRobot::Left ? frame_jacobian("left_foot", "local") :
                                                                      frame_jacobian("right_foot", "local");

  // IMU body Jacobian
  Eigen::MatrixXd J_IMU = frame_jacobian("trunk", "local");

  Eigen::MatrixXd J(6, J_contact.cols());
  J << J_contact.topRows(3), J_IMU.bottomRows(3);
  Eigen::MatrixXd J_u = J.leftCols(6);
  Eigen::MatrixXd J_a = J.rightCols(20);

  // XXX : is it better to use pseudo_invers or inverse ?
  Eigen::MatrixXd J_u_pinv = J_u.completeOrthogonalDecomposition().pseudoInverse();  // J_u.inverse();

  Eigen::VectorXd M(6);
  M << 0, 0, 0, omega_b;

  return J_u_C * J_u_pinv * M + (J_a_C - J_u_C * J_u_pinv * J_a) * qd_a;
}

Eigen::Vector2d HumanoidRobot::dcm(Eigen::Vector2d com_velocity, double omega)
{
  // DCM = c + (1/omega) c_dot
  return com_world().head(2) + (1 / omega) * com_velocity;
}

Eigen::Vector2d HumanoidRobot::zmp(Eigen::Vector2d com_acceleration, double omega)
{
  // ZMP = c - (1/omega^2) c_ddot
  return com_world().head(2) - (1 / pow(omega, 2)) * com_acceleration;
}

bool HumanoidRobot::camera_look_at(double& pan, double& tilt, const Eigen::Vector3d& P_world_target)
{
  // Compute view vector in head yaw frame
  Eigen::Affine3d T_world_headBase = get_T_world_frame("head_base");
  Eigen::Vector3d P_headBase_target = T_world_headBase.inverse() * P_world_target;

  // The pan is simply the angle in the XY plane
  pan = atan2(P_headBase_target.y(), P_headBase_target.x());

  // We then consider the (head_base x axis, head_pitch) plane
  Eigen::Vector2d P_headPitchPlane_target(sqrt(pow(P_headBase_target.x(), 2) + pow(P_headBase_target.y(), 2)),
                                          P_headBase_target.z() - dist_z_pan_tilt);

  double theta = M_PI / 2 - atan2(P_headPitchPlane_target.y(), P_headPitchPlane_target.x());

  // We just use beta = cos(opposed / hypothenus) to watch it with camera
  double ratio = dist_z_pan_camera / P_headPitchPlane_target.norm();
  if (ratio > 1 || ratio < -1)
  {
    return false;
  }
  double beta = acos(ratio);
  tilt = theta - beta;

  return true;
}

void HumanoidRobot::update_trunk_orientation(double roll, double pitch, double yaw)
{
  Eigen::Affine3d T_world_trunk = get_T_world_trunk();
  T_world_trunk.linear() = pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(roll, pitch, yaw));

  update_kinematics();
  set_T_world_frame(trunk, T_world_trunk);
  update_kinematics();
}

#ifdef HAVE_RHOBAN_UTILS
void HumanoidRobot::readFromHistories(rhoban_utils::HistoryCollection& histories, double timestamp, bool use_imu)
{
  // Updating DOFs from replay
  for (const std::string& name : actuated_joint_names())
  {
    set_joint(name, histories.number("read:" + name)->interpolate(timestamp));
  }

  // Set the support foot on the floor
  if (!use_imu)
  {
    double left_pressure = histories.number("left_pressure_weight")->interpolate(timestamp);
    double right_pressure = histories.number("right_pressure_weight")->interpolate(timestamp);
    if (left_pressure > right_pressure)
    {
      update_support_side(Left);
    }
    else
    {
      update_support_side(Right);
    }

    ensure_on_floor();
  }
#endif

  // Setting the trunk orientation from the IMU
  else
  {
    double imuYaw = histories.angle("imu_yaw")->interpolate(timestamp);
    double imuPitch = histories.angle("imu_pitch")->interpolate(timestamp);
    double imuRoll = histories.angle("imu_roll")->interpolate(timestamp);

    update_trunk_orientation(imuRoll, imuPitch, imuYaw);
  }
}
}  // namespace placo