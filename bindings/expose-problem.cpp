#include <iostream>
#include <pinocchio/fwd.hpp>
#include <sstream>
#include <eigenpy/eigenpy.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "doxystub.h"
#include "placo/problem/problem.h"
#include "placo/problem/variable.h"
#include "placo/problem/expression.h"
#include "placo/problem/constraint.h"
#include "placo/problem/polygon_constraint.h"
#include "placo/problem/integrator.h"
#include "placo/problem/problem_polynom.h"
#include "placo/problem/sparsity.h"
#include "placo/problem/qp_error.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace boost::python;
using namespace placo;
using namespace placo::problem;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(expr_overloads, expr, 0, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(integrator_expr_overloads, expr, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(integrator_expr_t_overloads, expr_t, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(problem_polynom_expr_overloads, expr, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(configure_overloads, configure, 1, 2);

void exposeProblem()
{
  class__<QPError>("QPError", init<std::string>()).def("what", &QPError::what);

  class__<Sparsity::Interval>("SparsityInterval")
      .add_property("start", &Sparsity::Interval::start, &Sparsity::Interval::start)
      .add_property("end", &Sparsity::Interval::end, &Sparsity::Interval::end);

  class__<Sparsity>("Sparsity")
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

  class__<ProblemConstraint>("ProblemConstraint")
      .add_property("expression", &ProblemConstraint::expression)
      .add_property(
          "priority",
          +[](const ProblemConstraint& constraint) {
            if (constraint.priority == ProblemConstraint::Hard)
            {
              return "hard";
            }
            else
            {
              return "soft";
            }
          })
      .add_property("weight", &ProblemConstraint::weight)
      .add_property("is_active", &ProblemConstraint::is_active)
      .def<void (ProblemConstraint::*)(std::string, double)>("configure", &ProblemConstraint::configure,
                                                             configure_overloads());

  class__<PolygonConstraint>("PolygonConstraint")
      .def("in_polygon", &PolygonConstraint::in_polygon)
      .staticmethod("in_polygon")
      .def("in_polygon_xy", &PolygonConstraint::in_polygon_xy)
      .staticmethod("in_polygon_xy");

  class__<Integrator>("Integrator", init<Variable&, Expression, int, double>())
      .def(init<Variable&, Eigen::VectorXd, Eigen::MatrixXd, double>())
      .def("upper_shift_matrix", &Integrator::upper_shift_matrix)
      .staticmethod("upper_shift_matrix")
      .add_property("t_start", &Integrator::t_start, &Integrator::t_start)
      .def_readonly("M", &Integrator::M)
      .def_readonly("A", &Integrator::A)
      .def_readonly("B", &Integrator::B)
      .def_readonly("final_transition_matrix", &Integrator::final_transition_matrix)
      .def("expr", &Integrator::expr, integrator_expr_overloads())
      .def("expr_t", &Integrator::expr_t, integrator_expr_t_overloads())
      .def("value", &Integrator::value)
      .def("get_trajectory", &Integrator::get_trajectory);

  class__<ProblemPolynom>("ProblemPolynom", init<Variable&>())
      .def("expr", &ProblemPolynom::expr, problem_polynom_expr_overloads())
      .def("get_polynom", &ProblemPolynom::get_polynom);

  class__<Integrator::Trajectory>("IntegratorTrajectory")
      .def("value", &Integrator::Trajectory::value)
      .def("duration", &Integrator::Trajectory::duration);

  class__<Problem>("Problem")
      .def("add_variable", &Problem::add_variable, return_internal_reference<>())
      .def("add_constraint", &Problem::add_constraint, return_internal_reference<>())
      .def("add_limit", &Problem::add_limit, return_internal_reference<>())
      .def("solve", &Problem::solve)
      .def("clear_variables", &Problem::clear_variables)
      .def("clear_constraints", &Problem::clear_constraints)
      .def("dump_status", &Problem::dump_status)
      .add_property("x", &Problem::x)
      .add_property("n_variables", &Problem::n_variables, &Problem::n_variables)
      .add_property("n_inequalities", &Problem::n_inequalities, &Problem::n_inequalities)
      .add_property("n_equalities", &Problem::n_equalities, &Problem::n_equalities)
      .add_property("free_variables", &Problem::free_variables, &Problem::free_variables)
      .add_property("determined_variables", &Problem::determined_variables, &Problem::determined_variables)
      .add_property("slack_variables", &Problem::slack_variables, &Problem::slack_variables)
      .add_property("use_sparsity", &Problem::use_sparsity, &Problem::use_sparsity)
      .add_property("rewrite_equalities", &Problem::rewrite_equalities, &Problem::rewrite_equalities)
      .add_property("regularization", &Problem::regularization, &Problem::regularization)
      .add_property(
          "slacks", +[](const Problem& problem) { return problem.slacks; });

  class__<Variable>("Variable")
      .add_property("k_start", &Variable::k_start)
      .add_property("k_end", &Variable::k_end)
      .add_property("value", &Variable::value)
      .def("expr", &Variable::expr, expr_overloads());

  class__<Expression>("Expression")
      .def_readwrite("A", &Expression::A)
      .def_readwrite("b", &Expression::b)
      .def("__len__", &Expression::rows)
      .def("is_scalar", &Expression::is_scalar)
      .def("is_constant", &Expression::is_constant)
      .def("slice", &Expression::slice)
      .def("rows", &Expression::rows)
      .def("cols", &Expression::cols)
      .def("value", &Expression::value)
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
      .def(self * self)
      .def(self + other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() + self)
      .def(self - other<Eigen::VectorXd>())
      .def(other<Eigen::VectorXd>() - self)
      .def(self + float())
      .def(float() + self)
      .def(self - float())
      .def(float() - self)
      .def(other<Eigen::MatrixXd>() * self)
      .def("left_multiply", &Expression::left_multiply)
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

  implicitly_convertible<Eigen::VectorXd, Expression>();
}