#include "placo/planning/swing_foot.h"
#include "eiquadprog/eiquadprog.hpp"

namespace placo
{
struct FactorCubicHermiteCurve
{
  FactorCubicHermiteCurve(Eigen::Vector3d p0, Eigen::Vector3d n0, Eigen::Vector3d p1, Eigen::Vector3d n1)
    : p0(p0), n0(n0), p1(p1), n1(n1)
  {
  }

  Eigen::Vector3d H_lambda(double t)
  {
    return t * (1 + t * (t - 2)) * n0;
  }

  Eigen::Vector3d H_mu(double t)
  {
    return t * t * (t - 1) * n1;
  };

  Eigen::Vector3d H_cst(double t)
  {
    return p0 + t * t * (3 - 2 * t) * (p1 - p0);
  };

  Eigen::Vector3d p0;
  Eigen::Vector3d n0;
  Eigen::Vector3d p1;
  Eigen::Vector3d n1;
};

SwingFoot::Trajectory SwingFoot::make_trajectory(double t_start, double t_end, double height, Eigen::Vector3d start,
                                                 Eigen::Vector3d target)
{
  Trajectory trajectory;
  trajectory.t_start = t_start;
  trajectory.t_end = t_end;

  Eigen::Vector3d n0(0., 0., 1.);
  Eigen::Vector3d n1(0., 0., 1.);

  double t_a = 1 / 4.;
  double t_b = 3 / 4.;

  FactorCubicHermiteCurve factors(start, n0, target, n1);

  double a0 = factors.H_lambda(t_a).dot(n0);
  double b0 = factors.H_mu(t_a).dot(n0);
  double c0 = (factors.H_cst(t_a) - start).dot(n0);

  double a1 = factors.H_lambda(t_b).dot(n1);
  double b1 = factors.H_mu(t_b).dot(n1);
  double c1 = (factors.H_cst(t_b) - start).dot(n1);

  Eigen::MatrixXd CE(0, 0);
  Eigen::VectorXd ce(0);

  Eigen::MatrixXd P(2, 2);
  P.setIdentity();
  Eigen::VectorXd q(2);
  q.setZero();

  Eigen::MatrixXd G(2, 2);
  G << a0, b0,  //
      a1, b1;
  Eigen::VectorXd h(2);
  h << c0 - height, c1 - height;

  Eigen::VectorXd x(2);

  Eigen::VectorXi activeSet;
  size_t activeSetSize;
  double result = eiquadprog::solvers::solve_quadprog(P, q, CE, ce, G.transpose(), h, x, activeSet, activeSetSize);

  if (result == std::numeric_limits<double>::infinity())
  {
    throw std::runtime_error("SwingFoot: Infeasible QP (check your equality and inequality constraints)");
  }

  trajectory.compute_abcd(start, n0 * x[0], target, n1 * x[1]);

  return trajectory;
}

void SwingFoot::Trajectory::compute_abcd(Eigen::Vector3d p0, Eigen::Vector3d m0, Eigen::Vector3d p1, Eigen::Vector3d m1)
{
  a = 2 * p0 - 2 * p1 + m0 + m1;
  b = -3 * p0 + 3 * p1 - 2 * m0 - m1;
  c = m0;
  d = p0;
}

Eigen::Vector3d SwingFoot::Trajectory::pos(double t_)
{
  double t = (t_ - t_start) / (t_end - t_start);
  if (t > 1)
  {
    t = 1;
  }
  double t_2 = t * t;
  double t_3 = t_2 * t;

  return a * t_3 + b * t_2 + c * t + d;
}

Eigen::Vector3d SwingFoot::Trajectory::vel(double t_)
{
  double t = (t_ - t_start) / (t_end - t_start);
  if (t > 1)
  {
    t = 1;
  }
  double t_2 = t * t;

  return (3 * a * t_2 + 2 * b * t + c) / (t_end - t_start);
}
}  // namespace placo