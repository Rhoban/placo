#include "expose-utils.hpp"
#include "module.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/numpy.hpp>

using namespace boost::python;
namespace np = boost::python::numpy;

// Wrapper to convert Affine3d to a numpy 4x4 matrix
struct Affine3d_to_np
{
  static PyObject* convert(Eigen::Affine3d const& T)
  {
    Py_intptr_t shape[2] = { 4, 4 };
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> M = T.matrix();

    PyObject* array = PyArray_SimpleNew(2, shape, (int)NPY_DOUBLE);

    std::memcpy(PyArray_DATA((PyArrayObject*)array), M.data(), 4 * 4 * sizeof(double));

    return array;
  }
};

void exposeEigen()
{
  // Vectors of points
  exposeStdVector<Eigen::Vector2d>("vector_Vector2d");
  exposeStdVector<Eigen::Vector3d>("vector_Vector3d");

  eigenpy::enableEigenPy();

  // Ensuring types are exposed
  // eigenpy::exposeType<Eigen::Vector2d>();
  // eigenpy::exposeType<Eigen::Vector3d>();
  // eigenpy::exposeType<Eigen::VectorXd>();

  // Enables eigen for specific matrix sizes
  eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 4, 1>>();
  eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 6, 1>>();
  eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 9, 1>>();

  // Thanks to this, Affine3d will be seamlessly converted from/to numpy 4x4 matrices
  implicitly_convertible<Eigen::Matrix4d, Eigen::Affine3d>();
  to_python_converter<Eigen::Affine3d, Affine3d_to_np, false>();
}