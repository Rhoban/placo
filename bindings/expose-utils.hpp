#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <vector>

using namespace boost::python;

/**
 * @brief Exposes a given type for std::vector
 * @tparam T type
 * @param name name of the type as exposed in Python
 */
template <typename T> void exposeStdVector(const std::string &class_name) {
  typedef typename std::vector<T> vector_T;

  class_<vector_T>(class_name.c_str()).def(vector_indexing_suite<vector_T>());
}