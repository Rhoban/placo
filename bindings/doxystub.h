#pragma once

#include <boost/python.hpp>

/**
 * This wrapper adds a __cxx_class__ method on the exposed class, allowing to retrieve the original
 * C++ type of the object from Python.
 */
template <class W, class X1 = boost::python::detail::not_specified, class X2 = boost::python::detail::not_specified,
          class X3 = boost::python::detail::not_specified, typename... Args>
boost::python::class_<W, X1, X2, X3> class__(Args... args)
{
  return boost::python::class_<W, X1, X2>(args...).def(
      "__cxx_class__", +[]() { return boost::core::demangle(typeid(W).name()); });
}
