#include <iostream>
#include <pinocchio/fwd.hpp>
#include <sstream>
#include <eigenpy/eigenpy.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/problem/problem.h"
#include "placo/problem/variable.h"
#include "placo/problem/expression.h"
#include "placo/problem/constraint.h"
#include "placo/problem/polygon_constraint.h"
#include "placo/problem/integrator.h"
#include "placo/problem/sparsity.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(expr_overloads, expr, 0, 2);

void exposeProblem()
{
  class_<Sparsity::Interval>("SparsityInterval")
      .add_property("start", &Sparsity::Interval::start, &Sparsity::Interval::start)
      .add_property("end", &Sparsity::Interval::end, &Sparsity::Interval::end);

  class_<Sparsity>("Sparsity")
      .add_property(
          "intervals",
          +[](const Sparsity& sparsity) {
            Eigen::MatrixXi intervals(sparsity.intervals.size(), 2);
            int k = 0;
            for (auto& interval : sparsity.intervals)
            {
              intervals(k, 0) = interval.start;
              intervals(k, 1) = interval.end;
              k++;
            }

            return intervals;
          })
      .def(self + self)
      .def("add_interval", &Sparsity::add_interval)
      .def("detect_columns_sparsity", &Sparsity::detect_columns_sparsity)
      .def("print_intervals", &Sparsity::print_intervals)
      .staticmethod("detect_columns_sparsity");

  class_<ProblemConstraint>("ProblemConstraint")
      .add_property("expression", &ProblemConstraint::expression)
      .add_property("inequality", &ProblemConstraint::inequality)
      .add_property("hard", &ProblemConstraint::hard)
      .add_property("weight", &ProblemConstraint::weight)
      .def<void (ProblemConstraint::*)(std::string, double)>("configure", &ProblemConstraint::configure);

  class_<ProblemConstraints>("ProblemConstraints")
      .def<void (ProblemConstraints::*)(std::string, double)>("configure", &ProblemConstraints::configure);

  class_<PolygonConstraint>("PolygonConstraint")
      .def("add_polygon_constraint", &PolygonConstraint::add_polygon_constraint)
      .def("add_polygon_constraint_xy", &PolygonConstraint::add_polygon_constraint_xy)
      .staticmethod("add_polygon_constraint");

  class_<Integrator>("Integrator", init<Variable&, Eigen::VectorXd, int, double>())
      .def(init<Variable&, Eigen::VectorXd, Eigen::MatrixXd, double>())
      .def("continuous_system_matrix", &Integrator::continuous_system_matrix)
      .staticmethod("continuous_system_matrix")
      .add_property("t_start", &Integrator::t_start, &Integrator::t_start)
      .add_property(
          "M", +[](const Integrator& i) { return i.M; })
      .add_property(
          "A", +[](const Integrator& i) { return i.A; })
      .add_property(
          "B", +[](const Integrator& i) { return i.B; })
      .add_property(
          "final_transition_matrix", +[](const Integrator& i) { return i.final_transition_matrix; })
      .def("expr", &Integrator::expr)
      .def("expr_t", &Integrator::expr_t)
      .def("value", &Integrator::value)
      .def("get_trajectory", &Integrator::get_trajectory);

  class_<Integrator::Trajectory>("IntegratorTrajectory").def("value", &Integrator::Trajectory::value);

  class_<Problem>("Problem")
      .def("add_variable", &Problem::add_variable, return_internal_reference<>())
      .def("add_constraint", &Problem::add_constraint, return_internal_reference<>())
      .def("add_limit", &Problem::add_limit)
      .def("solve", &Problem::solve)
      .def("clear_variables", &Problem::clear_variables)
      .def("clear_constraints", &Problem::clear_constraints)
      .add_property("use_sparsity", &Problem::use_sparsity, &Problem::use_sparsity)
      .add_property(
          "slacks", +[](const Problem& problem) { return problem.slacks; });

  class_<Variable>("Variable")
      .add_property("k_start", &Variable::k_start)
      .add_property("k_end", &Variable::k_end)
      .add_property("name", &Variable::name, &Variable::name)
      .add_property("value", &Variable::value)
      .def("expr", &Variable::expr, expr_overloads());

  class_<Expression>("Expression")
      .add_property(
          "A", +[](Expression& e) { return e.A; })
      .add_property(
          "b", +[](Expression& e) { return e.b; })
      .def("__len__", &Expression::rows)
      .def("is_scalar", &Expression::is_scalar)
      .def("rows", &Expression::rows)
      .def("cols", &Expression::cols)
      .def("piecewise_add", &Expression::piecewise_add)
      .def("from_vector", &Expression::from_vector)
      .staticmethod("from_vector")
      .def("from_double", &Expression::from_double)
      .staticmethod("from_double")
      // Arithmetics
      .def(-self)
      .def(self / self)
      .def(self + self)
      .def(self - self)
      .def(self * float())
      .def(float() * self)
      .def(self + other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() + self)
      .def(self - other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() - self)
      .def(self + float())
      .def(float() + self)
      .def(self - float())
      .def(float() - self)
      .def(other<Eigen::MatrixXd>() * self)
      .def("multiply", &Expression::multiply)
      // Compare to build constraints
      .def(self >= self)
      .def(self <= self)
      .def(self == self)
      .def(self == other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() == self)
      .def(self == float())
      .def(float() == self)
      .def(self <= float())
      .def(float() <= self)
      .def(self >= float())
      .def(float() >= self)
      .def(self <= other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() <= self)
      .def(self >= other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() >= self)
      // Aggregation
      .def("sum", &Expression::sum)
      .def("mean", &Expression::mean);
}