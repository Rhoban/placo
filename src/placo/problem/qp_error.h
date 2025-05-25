#pragma once

#include <string>
#include <stdexcept>

namespace placo::problem
{
/**
 * @brief Exception raised by \ref Problem in case of failure
 */
class QPError : public std::runtime_error
{
public:
  QPError(std::string message = "");
};
}  // namespace placo::problem