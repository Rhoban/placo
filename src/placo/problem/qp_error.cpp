#include "placo/problem/qp_error.hpp"

namespace placo::problem {
QPError::QPError(std::string message)
    : std::runtime_error("QPError: " + message) {}
}; // namespace placo::problem