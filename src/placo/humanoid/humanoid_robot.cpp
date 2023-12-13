#include "placo/humanoid/humanoid_robot.h"
#include "placo/tools/utils.h"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace placo::humanoid
{
HumanoidRobot::HumanoidRobot(std::string model_directory, int flags, std::string urdf_content)
  : RobotWrapper(model_directory, flags, urdf_content)
{
  initialize();

  // Measuring some distances
  dist_y_trunk_foot = fabs(get_T_a_b("trunk", "left_hip_yaw").translation().y());

  if (model.existFrame("head_base") && model.existFrame("head_pitch") && model.existFrame("camera"))
  {
    dist_z_pan_tilt = get_T_a_b("head_base", "head_pitch").translation().z();
    dist_z_pan_camera = get_T_a_b("head_base", "camera").translation().z();
  }
  else
  {
    std::cerr << "WARNING: Can't find head frames in the model, camera_look_at won't work" << std::endl;
  }
}

void HumanoidRobot::initialize()
{
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

void HumanoidRobot::update_support_side(HumanoidRobot::Side new_side)
{
  if (new_side != support_side)
  {
    // Updating the support frame to this frame
    support_side = new_side;
    update_kinematics();

    T_world_support = tools::flatten_on_floor(get_T_world_frame(support_frame()));
  }
}

void HumanoidRobot::ensure_on_floor()
{
  // Updating the floating base so that the foot is where we want
  update_kinematics();
  set_T_world_frame(support_frame(), T_world_support);
  update_kinematics();
}

void HumanoidRobot::update_from_imu(Eigen::Matrix3d R_world_trunk)
{
  update_kinematics();

  Eigen::Affine3d T_trunk_support = get_T_a_b(trunk, support_frame());
  T_world_support.linear() = R_world_trunk * T_trunk_support.linear();

  set_T_world_frame(support_frame(), T_world_support);
  update_kinematics();
}

placo::model::RobotWrapper::FrameIndex HumanoidRobot::support_frame()
{
  return support_side == Left ? left_foot : right_foot;
}

placo::model::RobotWrapper::FrameIndex HumanoidRobot::flying_frame()
{
  return support_side == Left ? right_foot : left_foot;
}

void HumanoidRobot::update_support_side(const std::string& side)
{
  update_support_side(string_to_side(side));
}

Eigen::Vector3d HumanoidRobot::get_com_velocity(Side support, Eigen::Vector3d omega_b)
{
  // CoM Jacobians
  Eigen::Matrix3Xd J_C = com_jacobian();
  Eigen::Matrix3Xd J_u_C = J_C.leftCols(6);
  Eigen::Matrix3Xd J_a_C = J_C.rightCols(20);

  // Support foot
  Eigen::MatrixXd J_contact =
      support == Left ? frame_jacobian("left_foot", "local") : frame_jacobian("right_foot", "local");

  // IMU body Jacobian
  Eigen::MatrixXd J_IMU = frame_jacobian("trunk", "local");

  Eigen::MatrixXd J(6, J_contact.cols());
  J << J_contact.topRows(3), J_IMU.bottomRows(3);
  Eigen::MatrixXd J_u = J.leftCols(6);
  Eigen::MatrixXd J_a = J.rightCols(20);

  // XXX : is it better to use pseudo_invers or inverse ?
  Eigen::MatrixXd J_u_pinv = J_u.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::VectorXd M(6);
  M << 0, 0, 0, omega_b;

  return J_u_C * J_u_pinv * M + (J_a_C - J_u_C * J_u_pinv * J_a) * state.qd.block(6, 0, model.nv - 6, 1);
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

#ifdef HAVE_RHOBAN_UTILS
void HumanoidRobot::read_from_histories(rhoban_utils::HistoryCollection& histories, double timestamp,
                                        std::string source, bool use_imu)
{
  // Updating DOFs from replay
  for (const std::string& name : joint_names())
  {
    set_joint(name, histories.number(source + ":" + name)->interpolate(timestamp));
  }

  // Set the support
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

  // Setting the trunk orientation from the IMU
  if (use_imu)
  {
    double imuYaw = histories.angle("imu_yaw")->interpolate(timestamp);
    double imuPitch = histories.angle("imu_pitch")->interpolate(timestamp);
    double imuRoll = histories.angle("imu_roll")->interpolate(timestamp);

    Eigen::Matrix3d R_world_trunk = pinocchio::rpy::rpyToMatrix(imuRoll, imuPitch, imuYaw);
    update_from_imu(R_world_trunk);
  }
}
#endif
}  // namespace placo::humanoid