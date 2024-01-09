#include "placo/tools/prioritized.h"

namespace placo::tools
{
Prioritized::Priority priority_from_string(std::string priority)
{
  if (priority == "soft")
  {
    return Prioritized::Priority::Soft;
  }
  else if (priority == "hard")
  {
    return Prioritized::Priority::Hard;
  }
  else if (priority == "scaled")
  {
    return Prioritized::Priority::Scaled;
  }
  else
  {
    throw std::runtime_error(std::string("KinematicsSolver: Invalid priority: ") + priority);
  }
}

Prioritized::Prioritized()
{
  priority = Soft;
  weight = 1.0;
}

Prioritized::~Prioritized()
{
}

void Prioritized::configure(std::string name_, std::string priority_, double weight_)
{
  name = name_;
  priority = priority_from_string(priority_);
  weight = weight_;
}

void Prioritized::configure(std::string name_, Priority priority_, double weight_)
{
  name = name_;
  priority = priority_;
  weight = weight_;
}

std::string Prioritized::priority_name()
{
  switch (priority)
  {
    case Hard:
      return "hard";
    case Soft:
      return "soft";
    case Scaled:
      return "scaled";
    default:
      throw std::runtime_error("Prioritized::priority_name: Invalid priority");
  }
}
}  // namespace placo::tools