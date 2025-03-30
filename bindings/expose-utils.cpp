#include "expose-utils.hpp"

template <>
std::string exposed_name<std::string>()
{
  return "str";
}
template <>
std::string exposed_name<double>()
{
  return "float";
}
template <>
std::string exposed_name<float>()
{
  return "float";
}
template <>
std::string exposed_name<bool>()
{
  return "bool";
}
template <>
std::string exposed_name<int>()
{
  return "int";
}
template <>
std::string exposed_name<unsigned int>()
{
  return "int";
}
template <>
std::string exposed_name<short>()
{
  return "int";
}
template <>
std::string exposed_name<unsigned short>()
{
  return "int";
}
template <>
std::string exposed_name<char>()
{
  return "int";
}
template <>
std::string exposed_name<unsigned char>()
{
  return "int";
}
template <>
std::string exposed_name<long>()
{
  return "int";
}
template <>
std::string exposed_name<unsigned long>()
{
  return "int";
}

const char* np_docstring(std::string doc)
{
  std::string full_doc = doc + " [returns numpy.ndarray]";
  char* doc_with_type = new char[full_doc.length() + 1];
  sprintf(doc_with_type, "%s", full_doc.c_str());

  return doc_with_type;
}