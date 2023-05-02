#pragma once

#include <stdexcept>

namespace placo
{
class QPError : public std::runtime_error
{
public:
  QPError(std::string message = "");
};
}  // namespace placo