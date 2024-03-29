#include "placo/kinematics/com_polygon_constraint.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/problem/polygon_constraint.h"

namespace placo::kinematics
{
CoMPolygonConstraint::CoMPolygonConstraint(const std::vector<Eigen::Vector2d>& polygon, double margin)
  : polygon(polygon), margin(margin)
{
}

void CoMPolygonConstraint::add_constraint(placo::problem::Problem& problem)
{
  // Building the CoM expression
  Eigen::Vector2d com = solver->robot.com_world().topRows(2);
  Eigen::MatrixXd J = solver->robot.com_jacobian().topRows(2);

  problem::Expression com_xy;
  com_xy.A = J;
  if (dcm)
  {
    if (solver->dt == 0. || omega == 0.)
    {
      throw std::runtime_error("DCM mode requires a non-zero solver.dt and omega");
    }

    // DCM is d = c + dc / w with x = sqrt(g / h)
    // Future DCM is c + J dq + J dq / (dt * w)
    //             = c + J dq (1 + 1 / (dt * w))
    com_xy.A *= (1. + 1. / (solver->dt * omega));
  }
  com_xy.b = com;

  // Adding constraints
  problem.add_constraint(problem::PolygonConstraint::in_polygon_xy(com_xy, polygon, margin))
      .configure(priority == Prioritized::Priority::Hard ? problem::ProblemConstraint::Hard :
                                                           problem::ProblemConstraint::Soft,
                 weight);
}

}  // namespace placo::kinematics