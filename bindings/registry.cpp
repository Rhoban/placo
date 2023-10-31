#include <iostream>
#include <map>
#include "registry.h"

static std::map<std::string, std::string> registry_map;

void register_type(std::string cxx_type, std::string python_type)
{
  registry_map[boost::core::demangle(cxx_type.c_str())] = python_type;
}

void exposeRegistry()
{
  def(
      "get_classes_registry", +[]() {
        boost::python::dict dict;
        for (auto& pair : registry_map)
        {
          dict[pair.first] = pair.second;
        }
        return dict;
      });
}