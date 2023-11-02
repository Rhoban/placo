#pragma once

#include <boost/python.hpp>

using namespace boost::python;

void register_type(std::string cxx_type, std::string python_type);

template <class T, class DT>
class_<T> class__(const char* name, init_base<DT> const& init)
{
  class_<T> c = class_<T>(name, init);
  register_type(typeid(T).name(), boost::python::extract<std::string>(c.attr("__name__")));
  return c;
}

template <class T>
class_<T> class__(const char* name, no_init_t no = no_init)
{
  class_<T> c = class_<T>(name, no_init);
  register_type(typeid(T).name(), boost::python::extract<std::string>(c.attr("__name__")));
  return c;
}

template <class T, class X1, class DT>
class_<T, X1> class__(const char* name, init_base<DT> const& init)
{
  class_<T, X1> c = class_<T, X1>(name, init);
  register_type(typeid(T).name(), boost::python::extract<std::string>(c.attr("__name__")));
  return c;
}

template <class T, class X1>
class_<T, X1> class__(const char* name, no_init_t no = no_init)
{
  class_<T, X1> c = class_<T, X1>(name, no_init);
  register_type(typeid(T).name(), boost::python::extract<std::string>(c.attr("__name__")));
  return c;
}

template <class T, class X1, class X2, class DT>
class_<T, X1, X2> class__(const char* name, init_base<DT> const& init)
{
  class_<T, X1> c = class_<T, X1, X2>(name, init);
  register_type(typeid(T).name(), boost::python::extract<std::string>(c.attr("__name__")));
  return c;
}

template <class T, class X1, class X2>
class_<T, X1, X2> class__(const char* name, no_init_t no = no_init)
{
  class_<T, X1, X2> c = class_<T, X1, X2>(name, no_init);
  register_type(typeid(T).name(), boost::python::extract<std::string>(c.attr("__name__")));
  return c;
}

void exposeRegistry();