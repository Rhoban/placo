#include <iostream>
#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/planning/jerk_planner.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;



void exposeJerkPlanner() {
  enum_<JerkPlanner::ConstraintType>("ConstraintType")
      .value("position", JerkPlanner::ConstraintType::Position)
      .value("velocity", JerkPlanner::ConstraintType::Velocity)
      .value("acceleration", JerkPlanner::ConstraintType::Acceleration)
      .value("zmp", JerkPlanner::ConstraintType::ZMP)
      .value("dcm", JerkPlanner::ConstraintType::DCM);

  class_<JerkPlanner::Constraint>("Constraint")
      .add_property("A", &JerkPlanner::Constraint::A)
      .add_property("b", &JerkPlanner::Constraint::b);

  class_<JerkPlanner::JerkTrajectory2D>("JerkTrajectory2D", init<double>())
      .def("duration", &JerkPlanner::JerkTrajectory2D::duration)
      .def("pos", &JerkPlanner::JerkTrajectory2D::pos)
      .def("vel", &JerkPlanner::JerkTrajectory2D::vel)
      .def("acc", &JerkPlanner::JerkTrajectory2D::acc);

  class_<JerkPlanner>("JerkPlanner",
                      init<int, JerkPlanner::State, double, double>())
      .def("add_equality_constraint", &JerkPlanner::add_equality_constraint)
      .def("add_inequality_constraint", &JerkPlanner::add_inequality_constraint)
      .def("add_inequality_polygon_constraint",
           &JerkPlanner::add_inequality_polygon_constraint)
      .def("add_limit_constraint", &JerkPlanner::add_limit_constraint)
      .def("plan", &JerkPlanner::plan);
}