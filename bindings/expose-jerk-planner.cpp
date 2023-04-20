#include <iostream>
#include <pinocchio/fwd.hpp>
#include <sstream>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/planning/jerk_planner.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeJerkPlanner()
{
  enum_<JerkPlanner::ConstraintType>("ConstraintType")
      .value("position", JerkPlanner::ConstraintType::Position)
      .value("velocity", JerkPlanner::ConstraintType::Velocity)
      .value("acceleration", JerkPlanner::ConstraintType::Acceleration)
      .value("zmp", JerkPlanner::ConstraintType::ZMP)
      .value("dcm", JerkPlanner::ConstraintType::DCM)
      .value("dzmp", JerkPlanner::ConstraintType::dZMP)
      .value("jerk", JerkPlanner::ConstraintType::Jerk);

  class_<JerkPlanner::Constraint>("Constraint", init<JerkPlanner&>())
      .def("is_active", &JerkPlanner::Constraint::is_active);

  class_<JerkPlanner::EqualityConstraint>("EqualityConstraint", init<>())
      .def<void (JerkPlanner::EqualityConstraint::*)(std::string, double)>("configure",
                                                                           &JerkPlanner::EqualityConstraint::configure);

  class_<JerkPlanner::JerkTrajectory2D>("JerkTrajectory2D", init<double, double>())
      .def("duration", &JerkPlanner::JerkTrajectory2D::duration)
      .def("pos", &JerkPlanner::JerkTrajectory2D::pos)
      .def("vel", &JerkPlanner::JerkTrajectory2D::vel)
      .def("acc", &JerkPlanner::JerkTrajectory2D::acc)
      .def("zmp", &JerkPlanner::JerkTrajectory2D::zmp)
      .def("dzmp", &JerkPlanner::JerkTrajectory2D::dzmp)
      .def("jerk", &JerkPlanner::JerkTrajectory2D::jerk)
      .def("dcm", &JerkPlanner::JerkTrajectory2D::dcm);

  auto toString = +[](const JerkPlanner& planner) {
    std::ostringstream oss;
    oss << "[JerkPlanner]" << std::endl;
    oss << "* steps: " << planner.N << std::endl;
    // oss << "* equalities: " << planner.equalities_count << std::endl;
    oss << "* inequalities: " << planner.inequalities_count << std::endl;

    return oss.str();
  };

  class_<JerkPlanner>("JerkPlanner", init<int, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d, double, double>())
      .def("add_equality_constraint", &JerkPlanner::add_equality_constraint, return_internal_reference<>())
      .def("add_lower_than_constraint", &JerkPlanner::add_lower_than_constraint)
      .def("add_greater_than_constraint", &JerkPlanner::add_greater_than_constraint)
      .def("add_polygon_constraint", &JerkPlanner::add_polygon_constraint)
      .def("add_limit_constraint", &JerkPlanner::add_limit_constraint)
      .def("plan", &JerkPlanner::plan)
      .def("__str__", toString)
      .def("__repr__", toString);
}