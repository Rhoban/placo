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

// Allows Python lists and tuples (e.g. [0., 0., 1.]) to be passed where an Eigen matrix or vector is expected.
// Numpy arrays are still handled by eigenpy, this converter is only tried for other sequences.
template <typename MatrixType>
struct Eigen_from_sequence
{
  Eigen_from_sequence()
  {
    converter::registry::push_back(&convertible, &construct, type_id<MatrixType>());
  }

  // Converts the sequence to a float64 numpy array, or returns NULL if it can't be converted to MatrixType
  static PyArrayObject* to_array(PyObject* obj)
  {
    if (!PyList_Check(obj) && !PyTuple_Check(obj))
    {
      return NULL;
    }

    PyObject* array = PyArray_FROMANY(obj, NPY_DOUBLE, 1, 2, NPY_ARRAY_CARRAY | NPY_ARRAY_FORCECAST);
    if (array == NULL)
    {
      PyErr_Clear();
      return NULL;
    }

    int ndim = PyArray_NDIM((PyArrayObject*)array);
    npy_intp rows = PyArray_DIM((PyArrayObject*)array, 0);
    npy_intp cols = ndim == 2 ? PyArray_DIM((PyArrayObject*)array, 1) : 1;
    bool ok = (ndim == 2 || MatrixType::ColsAtCompileTime == 1 || MatrixType::RowsAtCompileTime == 1) &&
              (MatrixType::RowsAtCompileTime == Eigen::Dynamic || MatrixType::RowsAtCompileTime == rows) &&
              (MatrixType::ColsAtCompileTime == Eigen::Dynamic || MatrixType::ColsAtCompileTime == cols);
    if (!ok)
    {
      Py_DECREF(array);
      return NULL;
    }

    return (PyArrayObject*)array;
  }

  static void* convertible(PyObject* obj)
  {
    PyArrayObject* array = to_array(obj);
    if (array == NULL)
    {
      return NULL;
    }
    Py_DECREF(array);
    return obj;
  }

  static void construct(PyObject* obj, converter::rvalue_from_python_stage1_data* data)
  {
    PyArrayObject* array = to_array(obj);
    npy_intp rows = PyArray_DIM(array, 0);
    npy_intp cols = PyArray_NDIM(array) == 2 ? PyArray_DIM(array, 1) : 1;

    void* storage = ((converter::rvalue_from_python_storage<MatrixType>*)data)->storage.bytes;
    MatrixType* M = new (storage) MatrixType();
    *M = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        (double*)PyArray_DATA(array), rows, cols);
    Py_DECREF(array);

    data->convertible = storage;
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

  // Lists and tuples can be passed instead of numpy arrays
  Eigen_from_sequence<Eigen::Vector2d>();
  Eigen_from_sequence<Eigen::Vector3d>();
  Eigen_from_sequence<Eigen::Matrix<double, 4, 1>>();
  Eigen_from_sequence<Eigen::Matrix<double, 6, 1>>();
  Eigen_from_sequence<Eigen::Matrix<double, 9, 1>>();
  Eigen_from_sequence<Eigen::VectorXd>();
  Eigen_from_sequence<Eigen::Matrix3d>();
  Eigen_from_sequence<Eigen::Matrix4d>();
  Eigen_from_sequence<Eigen::MatrixXd>();

  // Thanks to this, Affine3d will be seamlessly converted from/to numpy 4x4 matrices
  implicitly_convertible<Eigen::Matrix4d, Eigen::Affine3d>();
  to_python_converter<Eigen::Affine3d, Affine3d_to_np, false>();
}