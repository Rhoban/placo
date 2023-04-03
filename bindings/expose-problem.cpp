#include <iostream>
#include <pinocchio/fwd.hpp>
#include <sstream>
#include <eigenpy/eigenpy.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/problem/problem.h"
#include "placo/problem/variable.h"
#include "placo/problem/expression.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(expr_overloads, expr, 0, 2);

void exposeProblem()
{
  class_<Problem::Constraint>("ProblemConstraint")
      .add_property("expression", &Problem::Constraint::expression)
      .add_property("inequality", &Problem::Constraint::inequality)
      .add_property("hard", &Problem::Constraint::hard)
      .add_property("weight", &Problem::Constraint::weight)
      .def("configure", &Problem::Constraint::configure);

  class_<Problem>("Problem")
      .def("add_variable", &Problem::add_variable, return_internal_reference<>())
      .def("add_equality_zero", &Problem::add_equality_zero, return_internal_reference<>())
      .def("add_equality", &Problem::add_equality, return_internal_reference<>())
      .def("add_greater_than_zero", &Problem::add_greater_than_zero, return_internal_reference<>())
      .def("add_greater_than", &Problem::add_greater_than, return_internal_reference<>())
      .def("add_lower_than_zero", &Problem::add_lower_than_zero, return_internal_reference<>())
      .def("add_lower_than", &Problem::add_lower_than, return_internal_reference<>())
      .def("add_limit", &Problem::add_limit)
      .def("solve", &Problem::solve);

  class_<Variable>("Variable")
      .add_property("k_start", &Variable::k_start)
      .add_property("k_end", &Variable::k_end)
      .add_property("name", &Variable::name)
      .add_property("value", &Variable::value)
      .def("expr", &Variable::expr, expr_overloads());

  class_<Expression>("Expression")
      .add_property(
          "A", +[](Expression& e) { return e.A; })
      .add_property(
          "b", +[](Expression& e) { return e.b; })
      .def("rows", &Expression::rows)
      .def("cols", &Expression::cols)
      .def(-self)
      .def(self << self)
      .def(self + self)
      .def(self - self)
      .def(self * float())
      .def(float() * self)
      .def(self + other<Eigen::MatrixXd>())
      .def(other<Eigen::MatrixXd>() + self)
      .def(self - other<Eigen::MatrixXd>())
      .def(other<Eigen::MatrixXd>() - self)
      .def(other<Eigen::MatrixXd>() * self)
      .def("multiply", &Expression::multiply)
      .def("sum", &Expression::sum)
      .def("mean", &Expression::mean);
}