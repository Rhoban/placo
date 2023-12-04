#include "placo/humanoid/swing_foot_quintic.h"
#include "eiquadprog/eiquadprog.hpp"

namespace placo::humanoid
{
Eigen::VectorXd quintic_pos_times(double t)
{
  Eigen::VectorXd times(18);

  times << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), pow(t, 1), 1;

  return times;
}

Eigen::VectorXd quintic_d_times(double t)
{
  Eigen::VectorXd times(18);

  times << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * pow(t, 1), 1, 0;

  return times;
}

Eigen::VectorXd quintic_dd_times(double t)
{
  Eigen::VectorXd times(18);

  times << 20 * pow(t, 3), 12 * pow(t, 2), 6 * pow(t, 1), 2, 0, 0;

  return times;
}

SwingFootQuintic::Trajectory SwingFootQuintic::make_trajectory(double t_start, double t_end, double height,
                                                               Eigen::Vector3d start, Eigen::Vector3d target)
{
  Trajectory trajectory;

  double t_a = t_start + (t_end - t_start) / 4.;
  double t_b = t_start + 3 * (t_end - t_start) / 4.;

  // Building constraint matrices
  Eigen::MatrixXd A(18, 18);
  Eigen::VectorXd b(18);

  A.setZero();
  b.setZero();

  // Initial conditions
  A.block(0, 0, 1, 6) = quintic_pos_times(t_start).transpose();
  A.block(1, 6, 1, 6) = quintic_pos_times(t_start).transpose();
  A.block(2, 12, 1, 6) = quintic_pos_times(t_start).transpose();
  b.block(0, 0, 3, 1) = start;

  // Final conditions
  A.block(3, 0, 1, 6) = quintic_pos_times(t_end).transpose();
  A.block(4, 6, 1, 6) = quintic_pos_times(t_end).transpose();
  A.block(5, 12, 1, 6) = quintic_pos_times(t_end).transpose();
  b.block(3, 0, 3, 1) = target;

  // Initial and final velocity are zero
  A.block(6, 0, 1, 6) = quintic_d_times(t_start).transpose();
  A.block(7, 6, 1, 6) = quintic_d_times(t_start).transpose();
  A.block(8, 12, 1, 6) = quintic_d_times(t_start).transpose();
  A.block(9, 0, 1, 6) = quintic_d_times(t_end).transpose();
  A.block(10, 6, 1, 6) = quintic_d_times(t_end).transpose();
  A.block(11, 12, 1, 6) = quintic_d_times(t_end).transpose();

  // Initial and final acceleration are zero for x and y
  A.block(12, 0, 1, 6) = quintic_dd_times(t_start).transpose();
  A.block(13, 6, 1, 6) = quintic_dd_times(t_start).transpose();
  A.block(14, 0, 1, 6) = quintic_dd_times(t_end).transpose();
  A.block(15, 6, 1, 6) = quintic_dd_times(t_end).transpose();

  // Height at t_a and t_b is height
  A.block(16, 12, 1, 6) = quintic_pos_times(t_a).transpose();
  b[16] = height;
  A.block(17, 12, 1, 6) = quintic_pos_times(t_b).transpose();
  b[17] = height;

  Eigen::VectorXd coeffs = A.inverse() * b;
  trajectory.a = Eigen::Vector3d(coeffs[0], coeffs[6], coeffs[12]);
  trajectory.b = Eigen::Vector3d(coeffs[1], coeffs[7], coeffs[13]);
  trajectory.c = Eigen::Vector3d(coeffs[2], coeffs[8], coeffs[14]);
  trajectory.d = Eigen::Vector3d(coeffs[3], coeffs[9], coeffs[15]);
  trajectory.e = Eigen::Vector3d(coeffs[4], coeffs[10], coeffs[16]);
  trajectory.f = Eigen::Vector3d(coeffs[5], coeffs[11], coeffs[17]);

  return trajectory;
}

Eigen::Vector3d SwingFootQuintic::Trajectory::pos(double t)
{
  double t_2 = t * t;
  double t_3 = t_2 * t;
  double t_4 = t_3 * t;
  double t_5 = t_4 * t;

  return a * t_5 + b * t_4 + c * t_3 + d * t_2 + e * t + f;
}

Eigen::Vector3d SwingFootQuintic::Trajectory::vel(double t)
{
  double t_2 = t * t;
  double t_3 = t_2 * t;
  double t_4 = t_3 * t;

  return 5 * a * t_4 + 4 * b * t_3 + 3 * c * t_2 + 2 * d * t + e;
}
}  // namespace placo::humanoid