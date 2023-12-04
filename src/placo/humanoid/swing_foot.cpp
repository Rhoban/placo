#include "placo/humanoid/swing_foot.h"
#include "eiquadprog/eiquadprog.hpp"

namespace placo::humanoid
{
Eigen::VectorXd position_coefficients(double t)
{
  double t_2 = t * t;
  double t_3 = t_2 * t;

  // ax^3 + bx^2 + cx + d
  Eigen::VectorXd v(4);
  v << t_3, t_2, t, 1;
  return v;
}

Eigen::VectorXd velocity_coefficients(double t)
{
  double t_2 = t * t;

  // 3ax^2 + 2bx + c
  Eigen::VectorXd v(4);
  v << 3 * t_2, 2 * t, 1, 0;
  return v;
}

SwingFoot::Trajectory SwingFoot::make_trajectory(double t_start, double t_end, double height, Eigen::Vector3d start,
                                                 Eigen::Vector3d target)
{
  Trajectory trajectory;
  trajectory.t_start = t_start;
  trajectory.t_end = t_end;

  // Constraining the starting and ending position, and the starting
  // and ending velocities.
  Eigen::MatrixXd A(4, 4);
  A.setZero();
  A << position_coefficients(0),               //
      position_coefficients(t_end - t_start),  //
      velocity_coefficients(0),                //
      velocity_coefficients(t_end - t_start);
  A = A.transpose().inverse();

  Eigen::VectorXd x(4);
  x << start.x(), target.x(), 0., 0.;
  Eigen::VectorXd abcd_x = A * x;

  Eigen::VectorXd y(4);
  y << start.y(), target.y(), 0., 0.;
  Eigen::VectorXd abcd_y = A * y;

  // Constraining position when starting, 1/4, 3/4 and ending
  Eigen::MatrixXd B(4, 4);
  B << position_coefficients(0),                             //
      position_coefficients((1. / 4.) * (t_end - t_start)),  //
      position_coefficients((3. / 4.) * (t_end - t_start)),  //
      position_coefficients(t_end - t_start);                //
  B = B.transpose().inverse();

  Eigen::VectorXd z(4);
  z << start.z(), height, height, target.z();
  Eigen::VectorXd abcd_z = B * z;

  trajectory.a = Eigen::Vector3d(abcd_x[0], abcd_y[0], abcd_z[0]);
  trajectory.b = Eigen::Vector3d(abcd_x[1], abcd_y[1], abcd_z[1]);
  trajectory.c = Eigen::Vector3d(abcd_x[2], abcd_y[2], abcd_z[2]);
  trajectory.d = Eigen::Vector3d(abcd_x[3], abcd_y[3], abcd_z[3]);

  return trajectory;
}

SwingFoot::Trajectory SwingFoot::remake_trajectory(SwingFoot::Trajectory& old_trajectory, double t,
                                                   Eigen::Vector3d target)
{
  Trajectory trajectory;
  trajectory.t_start = old_trajectory.t_start;
  trajectory.t_end = old_trajectory.t_end;

  // We ensure the given timepoint is preserved and replan the landing
  Eigen::MatrixXd A(4, 4);
  A.setZero();
  A << position_coefficients(t - trajectory.t_start),                //
      position_coefficients(trajectory.t_end - trajectory.t_start),  //
      velocity_coefficients(t - trajectory.t_start),                 //
      velocity_coefficients(trajectory.t_end - trajectory.t_start);
  A = A.transpose().inverse();

  Eigen::VectorXd x(4);
  x << old_trajectory.pos(t).x(), target.x(), old_trajectory.vel(t).x(), 0;
  Eigen::VectorXd abcd_x = A * x;

  Eigen::VectorXd y(4);
  y << old_trajectory.pos(t).y(), target.y(), old_trajectory.vel(t).y(), 0;
  Eigen::VectorXd abcd_y = A * y;

  trajectory.a = Eigen::Vector3d(abcd_x[0], abcd_y[0], old_trajectory.a.z());
  trajectory.b = Eigen::Vector3d(abcd_x[1], abcd_y[1], old_trajectory.b.z());
  trajectory.c = Eigen::Vector3d(abcd_x[2], abcd_y[2], old_trajectory.c.z());
  trajectory.d = Eigen::Vector3d(abcd_x[3], abcd_y[3], old_trajectory.d.z());

  return trajectory;
}

Eigen::Vector3d SwingFoot::Trajectory::pos(double t)
{
  t -= t_start;
  double t_2 = t * t;
  double t_3 = t_2 * t;

  return a * t_3 + b * t_2 + c * t + d;
}

Eigen::Vector3d SwingFoot::Trajectory::vel(double t)
{
  t -= t_start;
  double t_2 = t * t;

  return 3 * a * t_2 + 2 * b * t + c;
}
}  // namespace placo::humanoid