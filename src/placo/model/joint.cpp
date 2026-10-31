#include "placo/model/joint.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace placo::model
{
Joint::Joint(Type type, const Eigen::Vector3d& axis_) : type(type), axis(axis_), axis_index(-1)
{
  // Axes aligned with x, y or z have exact transformations
  if (type == REVOLUTE || type == REVOLUTE_UNBOUNDED || type == PRISMATIC)
  {
    for (int k = 0; k < 3; k++)
    {
      if (axis.isApprox(Eigen::Vector3d::Unit(k)))
      {
        axis_index = k;
        axis = Eigen::Vector3d::Unit(k);
      }
    }
    if (axis_index < 0)
    {
      axis.normalize();
    }
  }

  switch (type)
  {
    case UNIVERSE:
      nq = nv = 0;
      break;
    case FREE_FLYER:
      nq = 7;
      nv = 6;
      break;
    case REVOLUTE:
    case PRISMATIC:
      nq = nv = 1;
      break;
    case REVOLUTE_UNBOUNDED:
      nq = 2;
      nv = 1;
      break;
    case SPHERICAL:
      nq = 4;
      nv = 3;
      break;
  }

  S = Matrix6Xd::Zero(6, nv);
  if (type == FREE_FLYER)
  {
    S.setIdentity();
  }
  else if (type == REVOLUTE || type == REVOLUTE_UNBOUNDED)
  {
    S.col(0).tail<3>() = axis;
  }
  else if (type == PRISMATIC)
  {
    S.col(0).head<3>() = axis;
  }
  else if (type == SPHERICAL)
  {
    S.bottomRows<3>().setIdentity();
  }
}

Eigen::Matrix3d Joint::axis_rotation(double c, double s) const
{
  Eigen::Matrix3d R;
  if (axis_index == 0)
  {
    R << 1., 0., 0., 0., c, -s, 0., s, c;
  }
  else if (axis_index == 1)
  {
    R << c, 0., s, 0., 1., 0., -s, 0., c;
  }
  else if (axis_index == 2)
  {
    R << c, -s, 0., s, c, 0., 0., 0., 1.;
  }
  else
  {
    // Rotation about an arbitrary (unit) axis
    const Eigen::Vector3d sin_axis = s * axis;
    const Eigen::Vector3d cos1_axis = (1. - c) * axis;

    double tmp = cos1_axis.x() * axis.y();
    R(0, 1) = tmp - sin_axis.z();
    R(1, 0) = tmp + sin_axis.z();

    tmp = cos1_axis.x() * axis.z();
    R(0, 2) = tmp + sin_axis.y();
    R(2, 0) = tmp - sin_axis.y();

    tmp = cos1_axis.y() * axis.z();
    R(1, 2) = tmp - sin_axis.x();
    R(2, 1) = tmp + sin_axis.x();

    R.diagonal() = (cos1_axis.cwiseProduct(axis)).array() + c;
  }
  return R;
}

Eigen::Affine3d Joint::transform(const Eigen::VectorXd& q) const
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();

  switch (type)
  {
    case UNIVERSE:
      break;
    case FREE_FLYER: {
      Eigen::Quaterniond quaternion(q[idx_q + 6], q[idx_q + 3], q[idx_q + 4], q[idx_q + 5]);
      T.linear() = quaternion.toRotationMatrix();
      T.translation() = q.segment<3>(idx_q);
      break;
    }
    case REVOLUTE:
      T.linear() = axis_rotation(std::cos(q[idx_q]), std::sin(q[idx_q]));
      break;
    case REVOLUTE_UNBOUNDED:
      T.linear() = axis_rotation(q[idx_q], q[idx_q + 1]);
      break;
    case PRISMATIC:
      T.translation() = axis * q[idx_q];
      break;
    case SPHERICAL: {
      Eigen::Quaterniond quaternion(q[idx_q + 3], q[idx_q], q[idx_q + 1], q[idx_q + 2]);
      T.linear() = quaternion.toRotationMatrix();
      break;
    }
  }

  return T;
}

const Matrix6Xd& Joint::motion_subspace() const
{
  return S;
}

void Joint::neutral(Eigen::VectorXd& q) const
{
  switch (type)
  {
    case UNIVERSE:
      break;
    case FREE_FLYER:
      q.segment<7>(idx_q) << 0., 0., 0., 0., 0., 0., 1.;
      break;
    case REVOLUTE:
    case PRISMATIC:
      q[idx_q] = 0.;
      break;
    case REVOLUTE_UNBOUNDED:
      q[idx_q] = 1.;
      q[idx_q + 1] = 0.;
      break;
    case SPHERICAL:
      q.segment<4>(idx_q) << 0., 0., 0., 1.;
      break;
  }
}

void Joint::integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v, Eigen::VectorXd& q_out) const
{
  switch (type)
  {
    case UNIVERSE:
      break;
    case REVOLUTE:
    case PRISMATIC:
      q_out[idx_q] = q[idx_q] + v[idx_v];
      break;
    case REVOLUTE_UNBOUNDED: {
      // Rotating the unit complex number, then normalizing it (first order)
      const double c = q[idx_q], s = q[idx_q + 1];
      const double cos_v = std::cos(v[idx_v]), sin_v = std::sin(v[idx_v]);
      Eigen::Vector2d out(cos_v * c - sin_v * s, sin_v * c + cos_v * s);
      out *= (3 - out.squaredNorm()) / 2;
      q_out.segment<2>(idx_q) = out;
      break;
    }
    case FREE_FLYER: {
      const Eigen::Quaterniond quaternion(q.segment<4>(idx_q + 3));
      Eigen::Vector3d translation;
      Eigen::Quaterniond rotation;
      exp6(v.segment<6>(idx_v), translation, rotation);

      q_out.segment<3>(idx_q) = quaternion * translation + q.segment<3>(idx_q);

      Eigen::Quaterniond result = quaternion * rotation;
      if (result.dot(quaternion) < 0)
      {
        result.coeffs() = -result.coeffs();
      }
      first_order_normalize(result);
      q_out.segment<4>(idx_q + 3) = result.coeffs();
      break;
    }
    case SPHERICAL: {
      const Eigen::Quaterniond quaternion(q.segment<4>(idx_q));
      Eigen::Quaterniond result = quaternion * quaternion_exp3(v.segment<3>(idx_v));
      first_order_normalize(result);
      q_out.segment<4>(idx_q) = result.coeffs();
      break;
    }
  }
}

void Joint::difference(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1, Eigen::VectorXd& d) const
{
  switch (type)
  {
    case UNIVERSE:
      break;
    case REVOLUTE:
    case PRISMATIC:
      d[idx_v] = q1[idx_q] - q0[idx_q];
      break;
    case REVOLUTE_UNBOUNDED: {
      // Angle of R0^T R1
      const double cos_value = q0.segment<2>(idx_q).dot(q1.segment<2>(idx_q));
      const double sin_value = q0[idx_q] * q1[idx_q + 1] - q0[idx_q + 1] * q1[idx_q];
      d[idx_v] = std::atan2(sin_value, cos_value);
      break;
    }
    case FREE_FLYER: {
      const Eigen::Quaterniond quaternion0(q0.segment<4>(idx_q + 3));
      const Eigen::Quaterniond quaternion1(q1.segment<4>(idx_q + 3));
      const Eigen::Vector3d translation =
          quaternion0.conjugate() * Eigen::Vector3d(q1.segment<3>(idx_q) - q0.segment<3>(idx_q));
      d.segment<6>(idx_v) = log6(quaternion0.conjugate() * quaternion1, translation);
      break;
    }
    case SPHERICAL: {
      const Eigen::Quaterniond quaternion0(q0.segment<4>(idx_q));
      const Eigen::Quaterniond quaternion1(q1.segment<4>(idx_q));
      double theta;
      d.segment<3>(idx_v) = quaternion_log3(quaternion0.conjugate() * quaternion1, theta);
      break;
    }
  }
}

// Random coordinates within limits (as pinocchio, rand() is used, so that seeding with srand() gives the same
// configurations)
static void random_vector(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, int start, int size,
                          Eigen::VectorXd& q)
{
  const double infinity = std::numeric_limits<double>::infinity();
  for (int k = start; k < start + size; k++)
  {
    if (lower[k] == -infinity || upper[k] == infinity)
    {
      throw std::range_error("Joint::random: non bounded limit, cannot uniformly sample joint at rank " +
                             std::to_string(k));
    }
    q[k] = lower[k] + ((upper[k] - lower[k]) * rand()) / RAND_MAX;
  }
}

// Uniformly sampled unit quaternion
static Eigen::Quaterniond random_quaternion()
{
  const double u1 = (double)rand() / RAND_MAX;
  const double u2 = (double)rand() / RAND_MAX;
  const double u3 = (double)rand() / RAND_MAX;

  const double mult1 = std::sqrt(1 - u1);
  const double mult2 = std::sqrt(u1);

  Eigen::Quaterniond quaternion;
  quaternion.w() = mult1 * std::sin(2 * M_PI * u2);
  quaternion.x() = mult1 * std::cos(2 * M_PI * u2);
  quaternion.y() = mult2 * std::sin(2 * M_PI * u3);
  quaternion.z() = mult2 * std::cos(2 * M_PI * u3);
  return quaternion;
}

void Joint::random(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, Eigen::VectorXd& q) const
{
  switch (type)
  {
    case UNIVERSE:
      break;
    case REVOLUTE:
    case PRISMATIC:
      random_vector(lower, upper, idx_q, 1, q);
      break;
    case REVOLUTE_UNBOUNDED: {
      const double angle = -M_PI + 2 * M_PI * ((double)rand()) / RAND_MAX;
      q[idx_q] = std::cos(angle);
      q[idx_q + 1] = std::sin(angle);
      break;
    }
    case FREE_FLYER:
      random_vector(lower, upper, idx_q, 3, q);
      q.segment<4>(idx_q + 3) = random_quaternion().coeffs();
      break;
    case SPHERICAL:
      q.segment<4>(idx_q) = random_quaternion().coeffs();
      break;
  }
}

std::string Joint::type_name() const
{
  switch (type)
  {
    case UNIVERSE:
      return "universe";
    case FREE_FLYER:
      return "free_flyer";
    case REVOLUTE:
      return "revolute";
    case REVOLUTE_UNBOUNDED:
      return "revolute_unbounded";
    case PRISMATIC:
      return "prismatic";
    case SPHERICAL:
      return "spherical";
  }
  return "unknown";
}
}  // namespace placo::model
