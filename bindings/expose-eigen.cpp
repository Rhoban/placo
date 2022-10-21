#include "expose-utils.hpp"
#include "module.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

using namespace boost::python;

void exposeAffine3d() {
  auto toString = +[](const Eigen::Affine3d &a) {
    std::ostringstream oss;
    oss << "[Affine3d]" << std::endl;
    oss << "t: " << a.translation().transpose() << std::endl;
    oss << "R: " << std::endl << a.linear();
    return oss.str();
  };

  class_<Eigen::Affine3d>("Affine3d")
      .def(
          "from_matrix",
          +[](const Eigen::Matrix4d &m) {
            Eigen::Affine3d result;
            result.matrix() = m;
            return result;
          })
      .staticmethod("from_matrix")
      .add_property(
          "mat",
          +[](const Eigen::Affine3d &a) { return (Eigen::Matrix4d)a.matrix(); },
          +[](Eigen::Affine3d &a, const Eigen::Matrix4d &m) { a.matrix() = m; })
      .add_property(
          "t",
          +[](const Eigen::Affine3d &a) {
            return (Eigen::Vector3d)a.translation();
          },
          +[](Eigen::Affine3d &a, const Eigen::Vector3d &t) {
            a.translation() = t;
          })
      .add_property(
          "R",
          +[](const Eigen::Affine3d &a) { return (Eigen::Matrix3d)a.linear(); },
          +[](Eigen::Affine3d &a, const Eigen::Matrix3d &m) { a.linear() = m; })
      .def(
          "inv", +[](const Eigen::Affine3d &a) { return a.inverse(); })
      .def("__repr__", toString)
      .def("__str__", toString)
      .def(self * other<Eigen::Vector3d>())
      .def(self * self);

  // Vectors of points
  exposeStdVector<Eigen::Vector2d>("vector_Vector2d");
  exposeStdVector<Eigen::Vector3d>("vector_Vector3d");

  eigenpy::enableEigenPy();

  // Ensuring types are exposed
  eigenpy::exposeType<Eigen::Vector2d>();
  eigenpy::exposeType<Eigen::Vector3d>();

  // Enables eigen for specific matrix sizes
  eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 4, 1>>();
  eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 6, 1>>();
  eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 9, 1>>();
}