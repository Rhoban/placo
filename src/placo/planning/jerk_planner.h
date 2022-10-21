#pragma once

#include <Eigen/Dense>
#include <map>
#include <vector>
#include <algorithm>

namespace placo
{
/**
 * @brief The Jerk planner can be used to produce min-jerk (2D) trajectories with given constraints
 */
class JerkPlanner
{
public:
  // A trajectory produces by the planner
  struct JerkTrajectory
  {
    std::vector<Eigen::Vector3d> pos_vel_acc;
    std::vector<double> jerk;
    double dt;

    int get_offset(double t) const;
    double pos(double t) const;
    double vel(double t) const;
    double acc(double t) const;

    double duration() const;
  };

  // A 2D trajectory produced by the planner (combines x and y trajectoires)
  struct JerkTrajectory2D
  {
    JerkTrajectory2D(double dt);
    JerkTrajectory X;
    JerkTrajectory Y;
    double dt;

    double duration() const;

    Eigen::Vector2d pos(double t) const;
    Eigen::Vector2d vel(double t) const;
    Eigen::Vector2d acc(double t) const;
  };

  /**
   * @brief A state here is x, xd, xdd, y, yd, ydd (resp. position, velocity and acceleration of x and y)
   */
  typedef Eigen::Matrix<double, 6, 1> State;

  JerkPlanner(int nb_steps, State initial_state = State::Zero(), double dt = 0.1, double omega = 0.);

  /**
   * @brief sqrt(g/h): constant for pendulum-based points (ZMP and DCM)
   */
  double omega;

  /**
   * @brief Number of steps. Since we produce a jerk trajectory for x and y, the QP problem will be of
   * size 2*N
   */
  int N;

  /**
   * @brief time per step [s]
   */
  double dt;

  /**
   * @brief System matrix x_{k+1} = A x_k + B u_k
   */
  Eigen::Matrix<double, 3, 3> A;
  Eigen::Matrix<double, 3, 1> B;

  struct Constraint
  {
    // Represents block for a constraint, can be
    // "Ax + b = 0" or "Ax + b >= 0"
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;

    int nb_constraints();
  };

  // Types of constraints that can be used
  enum ConstraintType
  {
    Position = 1,
    Velocity = 2,
    Acceleration = 3,
    ZMP = 4,
    DCM = 5
  };

  State initial_state;

  /**
   * @brief Adds an equality constraint (Ax = b)
   * @param step the step when the equality constraint should be added
   * @param value the value for equality
   * @param type the type of value (see ConstraintType) to constrain
   */
  void add_equality_constraint(int step, Eigen::Vector2d value, ConstraintType type = ConstraintType::Position);

  /**
   * @brief Adds an inequality constraint (Ax >= b)
   * @param step the step when the inequality constraint should be added
   * @param value the value for equality
   * @param type the type of value (see ConstraintType) to constrain
   * @param greater if true, makes Ax <= b instead of Ax >= b
   */
  void add_inequality_constraint(int step, Eigen::Vector2d value, ConstraintType type = ConstraintType::Position, bool lower = false);

  /**
   * @brief Given a polygon, produces inequalities so that the given point lies inside the polygon.
   *        WARNING: Polygon must be clockwise (meaning that the exterior of the shape is on the trigonometric normal
   *        of the vertices)
   * @param step the step when the inequality should be added
   * @param polygon the (2D) polygon to use
   * @param type the type of value (see ConstraintType) to constrain
   * @param margin the margin (a positive value means that we are more "inside" the polygon)
   */
  void add_inequality_polygon_constraint(int step, std::vector<Eigen::Vector2d> polygon,
                                         ConstraintType type = ConstraintType::Position, double margin = 0.);

  /**
   * @brief Produces inequalities constraint so that a given value is limited in absolute value to be below the limit
   * @param limit the limit
   * @param type the type of value that should be constrained
   */
  void add_limit_constraint(double limit, ConstraintType type = ConstraintType::Velocity);

  /**
   * @brief Runs the solver and produces a JerkTrajectory2D solution
   * @return the produced trajectory
   */
  JerkPlanner::JerkTrajectory2D plan();

  /**
   * Creates constraint matrices Ax - b = 0 for a given step and point type
   */
  /**
   * @brief Produces constraint matrices A and B (so that Ax - b = 0) for a given step and point type
   * @param step the step
   * @param type the type of value (see ConstraintType) to constrain
   * @param value value for equality
   * @return a Constraint, that contains the value for the A and b matrices
   */
  Constraint make_constraint(int step, JerkPlanner::ConstraintType type,
                             Eigen::Vector2d value = Eigen::Vector2d::Zero());

protected:
  std::vector<Constraint> equalities;
  std::vector<Constraint> inequalities;

  int nb_constraints(std::vector<Constraint>& constraints);
  void stack_constraints(std::vector<Constraint>& constraints, Eigen::MatrixXd& As, Eigen::VectorXd& bs);

  // State matrix at the last step
  Eigen::MatrixXd final_transition_matrix;
  std::map<int, Eigen::MatrixXd> a_powers;
  void compute_transition_matrix();
  Eigen::MatrixXd get_transition_matrix(int step);
};
}