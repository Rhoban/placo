#include <iostream>

#include "jerk_planner.h"
#include <eiquadprog/eiquadprog.hpp>

#include <cmath>

using namespace std;

namespace placo
{
int JerkPlanner::ConstraintMatrices::nb_constraints()
{
  return A.rows();
}

JerkPlanner::Constraint::Constraint(JerkPlanner& planner) : planner(planner)
{
}

bool JerkPlanner::Constraint::is_active()
{
  // std::cout << planner.active_set << std::endl;
  // std::cout << "Size: " << (int)planner.set_size << std::endl;
  // std::cout << "Range:" << first_row << "/" << last_row << std::endl;
  for (int k = 0; k < planner.set_size; k++)
  {
    int i = planner.active_set[k];
    if (i >= first_row && i < last_row)
    {
      return true;
    }
  }

  return false;
}

int JerkPlanner::JerkTrajectory::get_offset(double t) const
{
  int k = floor(t / dt);
  if (k < 0)
  {
    k = 0;
  }
  if (k >= pos_vel_acc.size())
  {
    k = pos_vel_acc.size() - 1;
  }

  return k;
}

double JerkPlanner::JerkTrajectory::pos(double t) const
{
  int k = get_offset(t);
  t -= k * dt;

  return t * (t * (t * ((1 / 6.) * jerk[k]) + (1 / 2.) * pos_vel_acc[k][2]) + pos_vel_acc[k][1]) + pos_vel_acc[k][0];
}

double JerkPlanner::JerkTrajectory::vel(double t) const
{
  int k = get_offset(t);
  t -= k * dt;

  return t * (t * ((1 / 2.) * jerk[k]) + pos_vel_acc[k][2]) + pos_vel_acc[k][1];
}

double JerkPlanner::JerkTrajectory::acc(double t) const
{
  int k = get_offset(t);
  t -= k * dt;

  return t * jerk[k] + pos_vel_acc[k][2];
}

JerkPlanner::JerkTrajectory2D::JerkTrajectory2D(double dt, double omega) : dt(dt), omega(omega)
{
  X.dt = dt;
  Y.dt = dt;
}

JerkPlanner::JerkTrajectory2D::JerkTrajectory2D() : dt(0.)
{
  X.dt = 0.;
  Y.dt = 0.;
}

double JerkPlanner::JerkTrajectory2D::get_dt()
{
  return dt;
}

void JerkPlanner::JerkTrajectory2D::set_dt(double dt_)
{
  dt = dt_;
  X.dt = dt;
  Y.dt = dt;
}

double JerkPlanner::JerkTrajectory::duration() const
{
  return pos_vel_acc.size() * dt;
}

double JerkPlanner::JerkTrajectory2D::duration() const
{
  return X.duration();
}

Eigen::Vector2d JerkPlanner::JerkTrajectory2D::pos(double t) const
{
  return Eigen::Vector2d(X.pos(t), Y.pos(t));
}

Eigen::Vector2d JerkPlanner::JerkTrajectory2D::vel(double t) const
{
  return Eigen::Vector2d(X.vel(t), Y.vel(t));
}

Eigen::Vector2d JerkPlanner::JerkTrajectory2D::acc(double t) const
{
  return Eigen::Vector2d(X.acc(t), Y.acc(t));
}

Eigen::Vector2d JerkPlanner::JerkTrajectory2D::zmp(double t) const
{
  // ZMP = c - (1/omega^2) c_ddot
  return pos(t) - (1 / pow(omega, 2)) * acc(t);
}

Eigen::Vector2d JerkPlanner::JerkTrajectory2D::dcm(double t) const
{
  // DCM = c + (1/omega) c_dot
  return pos(t) + (1 / omega) * vel(t);
}

JerkPlanner::JerkPlanner(int nb_dt, Eigen::Vector2d initial_position, Eigen::Vector2d initial_velocity,
                         Eigen::Vector2d initial_acceleration, double dt, double omega)
  : dt(dt), omega(omega)
{
  initial_state << initial_position.x(), initial_velocity.x(), initial_acceleration.x(), initial_position.y(),
      initial_velocity.y(), initial_acceleration.y();

  N = nb_dt;

  A.setZero();
  B.setZero();

  A << 1, dt, pow(dt, 2) / 2,  //
      0, 1, dt,                //
      0, 0, 1;

  B << pow(dt, 3) / 6,  //
      pow(dt, 2) / 2,   //
      dt;

  compute_transition_matrix();
}

void JerkPlanner::compute_transition_matrix()
{
  final_transition_matrix = Eigen::MatrixXd(6, N * 2);
  final_transition_matrix.setZero();

  Eigen::Matrix<double, 3, 3> Ak;
  Ak.setIdentity();
  a_powers[0] = Ak;

  for (int step = 0; step < N; step++)
  {
    final_transition_matrix.block(0, (N - step) * 2 - 2, 3, 1) = Ak * B;
    final_transition_matrix.block(3, (N - step) * 2 - 1, 3, 1) = Ak * B;
    Ak = A * Ak;
    a_powers[step + 1] = Ak;
  }
}

Eigen::MatrixXd JerkPlanner::get_transition_matrix(int step)
{
  Eigen::MatrixXd transition_matrix(6, N * 2);
  transition_matrix.setZero();

  if (step > 0)
  {
    transition_matrix.block(0, 0, 6, step * 2) = final_transition_matrix.block(0, (N - step) * 2, 6, step * 2);
  }

  return transition_matrix;
}

JerkPlanner::ConstraintMatrices JerkPlanner::make_constraint(int step, JerkPlanner::ConstraintType type,
                                                             Eigen::Vector2d value)
{
  ConstraintMatrices constraint;

  // Initializing A and b matrices
  constraint.A = Eigen::MatrixXd(2, N * 2);
  constraint.A.setZero();
  constraint.b = Eigen::MatrixXd(2, 1);
  constraint.b.setZero();

  // Retrieving transition matrix for step
  auto transition_matrix = get_transition_matrix(step);

  // Rows that should be used/combined in the constraints
  std::vector<std::pair<int, double>> rows;

  if (type == ConstraintType::Position)
  {
    rows.push_back(std::pair<int, double>(0, 1.0));
  }
  else if (type == ConstraintType::Velocity)
  {
    rows.push_back(std::pair<int, double>(1, 1.0));
  }
  else if (type == ConstraintType::Acceleration)
  {
    rows.push_back(std::pair<int, double>(2, 1.0));
  }
  else if (type == ConstraintType::ZMP)
  {
    // ZMP = c - (1/omega^2) c_ddot
    rows.push_back(std::pair<int, double>(0, 1.0));
    rows.push_back(std::pair<int, double>(2, -1 / pow(omega, 2)));
  }
  else if (type == ConstraintType::DCM)
  {
    // DCM = c + (1/omega) c_dot
    rows.push_back(std::pair<int, double>(0, 1.0));
    rows.push_back(std::pair<int, double>(1, 1 / omega));
  }

  // The value we want to be equal to
  constraint.b.block(0, 0, 1, 1) = -value.block(0, 0, 1, 1);
  constraint.b.block(1, 0, 1, 1) = -value.block(1, 0, 1, 1);

  for (auto& row : rows)
  {
    int row_offset = row.first;
    double row_multiplier = row.second;

    constraint.A.block(0, 0, 1, N * 2) += row_multiplier * transition_matrix.block(0 + row_offset, 0, 1, N * 2);
    constraint.A.block(1, 0, 1, N * 2) += row_multiplier * transition_matrix.block(3 + row_offset, 0, 1, N * 2);

    constraint.b.block(0, 0, 1, 1) +=
        row_multiplier * (a_powers[step].block(row_offset, 0, 1, 3) * initial_state.block(0, 0, 3, 1));
    constraint.b.block(1, 0, 1, 1) +=
        row_multiplier * (a_powers[step].block(row_offset, 0, 1, 3) * initial_state.block(3, 0, 3, 1));
  }

  return constraint;
}

void JerkPlanner::add_equality_constraint(int step, Eigen::Vector2d value, JerkPlanner::ConstraintType type)
{
  // if (type == Position)
  // {
  //   std::cout << "#########################" << std::endl
  //             << "COM constraint of step " << step << std::endl
  //             << "x : " << round(100 * value.x()) / 100 << std::endl
  //             << "y : " << round(100 * value.y()) / 100 << std::endl;
  // }

  // if (type == ZMP)
  // {
  //   std::cout << "#########################" << std::endl
  //             << "ZMP constraint of step " << step << std::endl
  //             << "x : " << round(100 * value.x()) / 100 << std::endl
  //             << "y : " << round(100 * value.y()) / 100 << std::endl;
  // }

  _push_equality(make_constraint(step, type, value));
}

JerkPlanner::Constraint JerkPlanner::add_greater_than_constraint(int step, Eigen::Vector2d value,
                                                                 JerkPlanner::ConstraintType type)
{
  Constraint range(*this);
  range.first_row = inequalities_count;
  auto constraint = make_constraint(step, type, value);
  _push_inequality(constraint);

  range.last_row = inequalities_count;
  return range;
}

JerkPlanner::Constraint JerkPlanner::add_lower_than_constraint(int step, Eigen::Vector2d value,
                                                               JerkPlanner::ConstraintType type)
{
  Constraint range(*this);
  range.first_row = inequalities_count;
  auto constraint = make_constraint(step, type, value);
  constraint.A = -constraint.A;
  constraint.b = -constraint.b;
  _push_inequality(constraint);

  range.last_row = inequalities_count;
  return range;
}

JerkPlanner::Constraint JerkPlanner::add_limit_constraint(double limit, JerkPlanner::ConstraintType type)
{
  Constraint range(*this);
  range.first_row = inequalities_count;

  for (int step = 0; step < N; step++)
  {
    // Ax <= b
    add_lower_than_constraint(step, Eigen::Vector2d(limit, limit), type);

    // Ax >= -b
    add_greater_than_constraint(step, -Eigen::Vector2d(limit, limit), type);
  }

  range.last_row = inequalities_count;
  return range;
}

JerkPlanner::Constraint JerkPlanner::add_polygon_constraint(int step, std::vector<Eigen::Vector2d> polygon,
                                                            JerkPlanner::ConstraintType type, double margin)
{
  Constraint range(*this);
  range.first_row = inequalities_count;

  ConstraintMatrices inequality;
  ConstraintMatrices Ab_matrices = make_constraint(step, type);

  inequality.A = Eigen::MatrixXd(polygon.size(), N * 2);
  inequality.A.setZero();
  inequality.b = Eigen::MatrixXd(polygon.size(), 1);
  inequality.b.setZero();

  for (int i = 0; i < polygon.size(); i++)
  {
    int j = (i + 1) % polygon.size();

    const Eigen::Vector2d& A = polygon[i];
    const Eigen::Vector2d& B = polygon[j];
    Eigen::Vector2d n;
    n << (B - A).y(), (A - B).x();
    n.normalize();

    auto transition_matrix = get_transition_matrix(step);

    inequality.A.row(i) = Ab_matrices.A.block(0, 0, 1, N * 2) * n.x() + Ab_matrices.A.block(1, 0, 1, N * 2) * n.y();
    inequality.b.row(i) = Ab_matrices.b.block(0, 0, 1, 1) * n.x() + Ab_matrices.b.block(1, 0, 1, 1) * n.y();
    inequality.b(i, 0) -= A.dot(n) + margin;
  }

  _push_inequality(inequality);

  range.last_row = inequalities_count;
  return range;
}

void JerkPlanner::stack_constraints(std::vector<ConstraintMatrices>& constraints, int constraints_count,
                                    Eigen::MatrixXd& As, Eigen::VectorXd& bs)
{
  As = Eigen::MatrixXd(constraints_count, N * 2);
  bs = Eigen::VectorXd(constraints_count);

  int k = 0;
  for (auto& constraint : constraints)
  {
    As.block(k, 0, constraint.nb_constraints(), N * 2) = constraint.A;
    bs.block(k, 0, constraint.nb_constraints(), 1) = constraint.b;
    k += constraint.nb_constraints();
  }
}

JerkPlanner::JerkTrajectory2D JerkPlanner::plan()
{
  // Quadratic term (we want to minimize the jerk)
  Eigen::MatrixXd G;
  Eigen::VectorXd g0;
  G = Eigen::MatrixXd(N * 2, N * 2);
  g0 = Eigen::VectorXd(N * 2);
  G.setIdentity();
  g0.setZero();

  // Stacking equality constraints
  Eigen::MatrixXd CE;
  Eigen::VectorXd ce0;
  stack_constraints(equalities, equalities_count, CE, ce0);

  // Stacking inequality constraints
  Eigen::MatrixXd CI;
  Eigen::VectorXd ci0;
  stack_constraints(inequalities, inequalities_count, CI, ci0);

  Eigen::VectorXd jerks(N * 2);
  jerks.setZero();

  double result =
      eiquadprog::solvers::solve_quadprog(G, g0, CE.transpose(), ce0, CI.transpose(), ci0, jerks, active_set, set_size);

  if (result == std::numeric_limits<double>::infinity())
  {
    throw std::runtime_error("CoM planner: Infeasible QP (check your inequality constraints)");
  }

  Eigen::VectorXd state_x(3);
  Eigen::VectorXd state_y(3);
  State state = initial_state;

  // Building the Jerk Trajectory from QP outputs
  JerkPlanner::JerkTrajectory2D trajectory(dt, omega);

  for (int step = 0; step < N; step++)
  {
    trajectory.X.pos_vel_acc.push_back(Eigen::Vector3d(state[0], state[1], state[2]));
    trajectory.X.jerk.push_back(jerks[step * 2]);
    trajectory.Y.pos_vel_acc.push_back(Eigen::Vector3d(state[3], state[4], state[5]));
    trajectory.Y.jerk.push_back(jerks[step * 2 + 1]);

    state.block(0, 0, 3, 1) = A * state.block(0, 0, 3, 1) + B * jerks[step * 2];
    state.block(3, 0, 3, 1) = A * state.block(3, 0, 3, 1) + B * jerks[step * 2 + 1];
  }

  return trajectory;
}

void JerkPlanner::_push_equality(ConstraintMatrices constraint)
{
  equalities.push_back(constraint);
  equalities_count += constraint.nb_constraints();
}

void JerkPlanner::_push_inequality(ConstraintMatrices constraint)
{
  inequalities.push_back(constraint);
  inequalities_count += constraint.nb_constraints();
}

}  // namespace placo