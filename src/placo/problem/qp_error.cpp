#include "placo/problem/qp_error.h"

namespace placo
{
QPError::QPError(std::string message) : std::runtime_error("QPError: " + message)
{
}
};  // namespace placo