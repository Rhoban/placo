#include "placo/model/spatial.h"
#include <cmath>
#include <limits>

namespace placo::model
{
// The formulas (and their Taylor expansions near singularities) are the ones of pinocchio, so that the results are
// the same up to rounding errors
static const double eps = std::numeric_limits<double>::epsilon();

// Precision below which a Taylor expansion of the given degree is used
static double taylor_precision(int degree)
{
  return std::pow(eps, 1. / (degree + 1.));
}

static const double taylor_precision_2 = taylor_precision(2);
static const double taylor_precision_3 = taylor_precision(3);

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0., -v.z(), v.y(), v.z(), 0., -v.x(), -v.y(), v.x(), 0.;
  return m;
}

Eigen::Affine3d inverse(const Eigen::Affine3d& T)
{
  Eigen::Affine3d T_inv = Eigen::Affine3d::Identity();
  T_inv.linear() = T.linear().transpose();
  T_inv.translation() = -(T_inv.linear() * T.translation());
  return T_inv;
}

Vector6d act(const Eigen::Affine3d& T, const Vector6d& v)
{
  Vector6d result;
  result.tail<3>() = T.linear() * v.tail<3>();
  result.head<3>() = T.linear() * v.head<3>() + T.translation().cross(result.tail<3>());
  return result;
}

Vector6d act_inv(const Eigen::Affine3d& T, const Vector6d& v)
{
  Vector6d result;
  result.head<3>() = T.linear().transpose() * (v.head<3>() - T.translation().cross(v.tail<3>()));
  result.tail<3>() = T.linear().transpose() * v.tail<3>();
  return result;
}

Vector6d act_force(const Eigen::Affine3d& T, const Vector6d& f)
{
  Vector6d result;
  result.head<3>() = T.linear() * f.head<3>();
  result.tail<3>() = T.linear() * f.tail<3>() + T.translation().cross(result.head<3>());
  return result;
}

Vector6d motion_cross(const Vector6d& v1, const Vector6d& v2)
{
  Vector6d result;
  result.head<3>() = v1.tail<3>().cross(v2.head<3>()) + v1.head<3>().cross(v2.tail<3>());
  result.tail<3>() = v1.tail<3>().cross(v2.tail<3>());
  return result;
}

Vector6d force_cross(const Vector6d& v, const Vector6d& f)
{
  Vector6d result;
  result.head<3>() = v.tail<3>().cross(f.head<3>());
  result.tail<3>() = v.tail<3>().cross(f.tail<3>()) + v.head<3>().cross(f.head<3>());
  return result;
}

Matrix6d action_matrix(const Eigen::Affine3d& T)
{
  Matrix6d X;
  X.topLeftCorner<3, 3>() = T.linear();
  X.topRightCorner<3, 3>() = skew(T.translation()) * T.linear();
  X.bottomLeftCorner<3, 3>().setZero();
  X.bottomRightCorner<3, 3>() = T.linear();
  return X;
}

Eigen::Matrix3d exp3(const Eigen::Vector3d& v)
{
  const double t2 = v.squaredNorm() + eps * eps;
  const double t = std::sqrt(t2);
  const double st = std::sin(t);
  double ct = std::cos(t);

  double alpha_vxvx, alpha_vx;
  if (t > taylor_precision_3)
  {
    alpha_vxvx = (1 - ct) / t2;
    alpha_vx = st / t;
  }
  else
  {
    alpha_vxvx = 0.5 - t2 / 24;
    alpha_vx = 1 - t2 / 6;
    ct = 1 - t2 / 2;
  }

  Eigen::Matrix3d R = alpha_vxvx * v * v.transpose();
  R(0, 1) -= alpha_vx * v[2];
  R(1, 0) += alpha_vx * v[2];
  R(0, 2) += alpha_vx * v[1];
  R(2, 0) -= alpha_vx * v[1];
  R(1, 2) -= alpha_vx * v[0];
  R(2, 1) += alpha_vx * v[0];
  R.diagonal().array() += ct;

  return R;
}

// Angle and axis of a rotation close to pi, computed from the column i0 of the matrix
static void theta_axis_near_pi(int i0, double value, const Eigen::Matrix3d& R, double& angle, Eigen::Vector3d& axis)
{
  const int i1 = (i0 + 1) % 3;
  const int i2 = (i0 + 2) % 3;

  const double s = std::sqrt(value + eps + eps * eps) * (R(i2, i1) >= R(i1, i2) ? 1. : -1.);
  axis[i0] = s / 2;
  axis[i1] = 1 / (2 * s) * (R(i1, i0) + R(i0, i1));
  axis[i2] = 1 / (2 * s) * (R(i2, i0) + R(i0, i2));
  const double w = 1 / (2 * s) * (R(i2, i1) - R(i1, i2));

  const double axis_norm = axis.norm();
  angle = 2 * std::atan2(axis_norm, w);
  axis /= axis_norm;
}

Eigen::Vector3d log3(const Eigen::Matrix3d& R_input, double& theta)
{
  // Re-orthonormalizing the rotation matrix
  Eigen::Matrix3d R;
  R.col(0) = R_input.col(0) / R_input.col(0).norm();
  R.col(1) = R_input.col(1) / R_input.col(1).norm();
  R.col(2) = R.col(0).cross(R.col(1));
  R.col(0) = R.col(1).cross(R.col(2));

  const double tr = R.trace();
  const double cos_value = (tr - 1) / 2;
  const double prec = taylor_precision_2;

  if (cos_value < -1 + prec)
  {
    // The angle is close to pi
    Eigen::Vector3d value = 2 * R.diagonal().array() - tr + 1;
    int i0 = 2;
    if (value[0] >= value[1])
    {
      i0 = (value[0] >= value[2]) ? 0 : 2;
    }
    else
    {
      i0 = (value[1] >= value[2]) ? 1 : 2;
    }

    Eigen::Vector3d axis;
    theta_axis_near_pi(i0, value[i0], R, theta, axis);
    return theta * axis;
  }

  double theta_nominal;
  if (tr <= 3 - prec)
  {
    theta_nominal = std::acos(cos_value);
  }
  else
  {
    theta_nominal = std::sqrt(2 * (1 - cos_value) + eps * eps);
  }

  const Eigen::Vector3d antisymmetric_R(0.5 * (R(2, 1) - R(1, 2)), 0.5 * (R(0, 2) - R(2, 0)),
                                        0.5 * (R(1, 0) - R(0, 1)));
  const double n2 = antisymmetric_R.squaredNorm();

  double t;
  if (theta_nominal >= prec)
  {
    t = theta_nominal / std::sin(theta_nominal);
  }
  else
  {
    t = 1. + n2 / 6 + n2 * n2 * 3 / 40;
  }

  theta = theta_nominal;
  return t * antisymmetric_R;
}

Eigen::Vector3d log3(const Eigen::Matrix3d& R)
{
  double theta;
  return log3(R, theta);
}

Eigen::Matrix3d jlog3(const Eigen::Matrix3d& R)
{
  double theta;
  Eigen::Vector3d log = log3(R, theta);

  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  const double st_1mct = st / (1 - ct);

  double alpha, diag_value;
  if (theta < taylor_precision_3)
  {
    alpha = 1. / 12 + theta * theta / 720;
    diag_value = 0.5 * (2 - theta * theta / 6);
  }
  else
  {
    alpha = 1 / (theta * theta) - st_1mct / (2 * theta);
    diag_value = 0.5 * (theta * st_1mct);
  }

  Eigen::Matrix3d J = alpha * log * log.transpose();
  J.diagonal().array() += diag_value;
  J += skew(0.5 * log);

  return J;
}

Eigen::Quaterniond quaternion_exp3(const Eigen::Vector3d& v)
{
  const double t2 = v.squaredNorm();
  const double t = std::sqrt(t2 + eps * eps);

  if (t2 > taylor_precision_3)
  {
    return Eigen::Quaterniond(Eigen::AngleAxisd(t, v / t));
  }

  // Order 4 Taylor expansion in theta
  Eigen::Quaterniond quaternion;
  const double t2_2 = t2 / 4;
  quaternion.vec() = 0.5 * (1 - t2_2 / 6 + t2_2 * t2_2 / 120) * v;
  quaternion.w() = 1 - t2_2 / 2 + t2_2 * t2_2 / 24;
  return quaternion;
}

Eigen::Vector3d quaternion_log3(const Eigen::Quaterniond& quaternion, double& theta)
{
  const double norm_squared = quaternion.vec().squaredNorm();
  const double norm = std::sqrt(norm_squared + eps * eps);
  const double sign = quaternion.w() >= 0 ? 1. : -1.;
  const double w = sign * quaternion.w();

  const double theta_2 = std::atan2(norm, w);
  const double y_x = norm / w;
  const double y_x_sq = norm_squared / (w * w);

  double inv_sinc;
  if (norm_squared < taylor_precision_2)
  {
    theta = 2. * (1 - y_x_sq / 3) * y_x;
    const double th2_2 = theta * theta / 4;
    inv_sinc = 2 * (1 + th2_2 / 6 + 7. / 360 * th2_2 * th2_2);
  }
  else
  {
    theta = 2. * theta_2;
    inv_sinc = theta / std::sin(theta_2);
  }

  return inv_sinc * sign * quaternion.vec();
}

void exp6(const Vector6d& v, Eigen::Vector3d& translation, Eigen::Quaterniond& quaternion)
{
  const Eigen::Vector3d linear = v.head<3>();
  const Eigen::Vector3d w = v.tail<3>();

  const double t2 = w.squaredNorm() + eps * eps;
  const double t = std::sqrt(t2);
  const double st = std::sin(t);
  const double ct = std::cos(t);
  const double inv_t2 = 1 / t2;

  double alpha_wxv, alpha_w2;
  if (t < taylor_precision_3)
  {
    alpha_wxv = 0.5 - t2 / 24;
    alpha_w2 = 1. / 6 - t2 / 120;
  }
  else
  {
    alpha_wxv = (1 - ct) * inv_t2;
    alpha_w2 = (t - st) * inv_t2 / t;
  }

  translation = linear + alpha_wxv * w.cross(linear) + alpha_w2 * w.cross(w.cross(linear));
  quaternion = quaternion_exp3(w);
}

Vector6d log6(const Eigen::Quaterniond& quaternion, const Eigen::Vector3d& translation)
{
  double theta;
  const Eigen::Vector3d w = quaternion_log3(quaternion, theta);
  const double t2 = w.squaredNorm();

  const double sign = quaternion.w() >= 0 ? 1. : -1.;
  const double ct_2 = sign * quaternion.w();
  const double st_2 = std::sqrt(quaternion.vec().squaredNorm() + eps * eps);
  const double cot_th_2 = ct_2 / st_2;
  const double th_2_squared = t2 / 4;

  double beta;
  if (theta <= taylor_precision_3)
  {
    beta = (1. / 3 - th_2_squared / 45) / 4;
  }
  else
  {
    beta = 1 / t2 - cot_th_2 * 0.5 / theta;
  }

  Vector6d result;
  result.head<3>() = translation - 0.5 * w.cross(translation) + beta * w.cross(w.cross(translation));
  result.tail<3>() = w;
  return result;
}

void first_order_normalize(Eigen::Quaterniond& quaternion)
{
  const double n2 = quaternion.squaredNorm();
  quaternion.coeffs() *= (3 - n2) / 2;
}

Eigen::Matrix3d rpy_to_matrix(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Vector3d matrix_to_rpy(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d rpy = R.eulerAngles(2, 1, 0).reverse();

  if (rpy[1] < -M_PI / 2)
  {
    rpy[1] += 2 * M_PI;
  }

  if (rpy[1] > M_PI / 2)
  {
    rpy[1] = M_PI - rpy[1];
    if (rpy[0] < 0)
    {
      rpy[0] += M_PI;
    }
    else
    {
      rpy[0] -= M_PI;
    }
    rpy[2] -= M_PI;
  }

  return rpy;
}

Inertia::Inertia(double mass, const Eigen::Vector3d& lever, const Eigen::Matrix3d& inertia)
  : mass(mass), lever(lever), inertia(inertia)
{
}

bool Inertia::is_zero() const
{
  return mass == 0. && lever.isZero(0.) && inertia.isZero(0.);
}

Inertia Inertia::transformed(const Eigen::Affine3d& T) const
{
  return Inertia(mass, T.translation() + T.linear() * lever, T.linear() * inertia * T.linear().transpose());
}

Inertia& Inertia::operator+=(const Inertia& other)
{
  const double total_mass = mass + other.mass;
  const double total_mass_inv = 1. / std::max(total_mass, eps);
  const Eigen::Vector3d AB = lever - other.lever;

  lever *= mass * total_mass_inv;
  lever += (other.mass * total_mass_inv) * other.lever;

  // Parallel axis theorem
  const Eigen::Matrix3d skew_AB = skew(AB);
  inertia += other.inertia - (mass * other.mass * total_mass_inv) * (skew_AB * skew_AB);
  mass = total_mass;

  return *this;
}

Matrix6d Inertia::matrix() const
{
  const Eigen::Matrix3d skew_c = skew(lever);

  Matrix6d M;
  M.topLeftCorner<3, 3>() = mass * Eigen::Matrix3d::Identity();
  M.topRightCorner<3, 3>() = -mass * skew_c;
  M.bottomLeftCorner<3, 3>() = mass * skew_c;
  M.bottomRightCorner<3, 3>() = inertia - mass * skew_c * skew_c;
  return M;
}

Vector6d Inertia::momentum(const Vector6d& v) const
{
  Vector6d f;
  f.head<3>() = mass * (v.head<3>() - lever.cross(v.tail<3>()));
  f.tail<3>() = inertia * v.tail<3>() + lever.cross(f.head<3>());
  return f;
}
}  // namespace placo::model
