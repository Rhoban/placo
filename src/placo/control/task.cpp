#include "placo/control/task.h"

namespace placo
{
Task::Priority priority_from_string(std::string priority)
{
  if (priority == "soft")
  {
    return Task::Priority::Soft;
  }
  else if (priority == "hard")
  {
    return Task::Priority::Hard;
  }
  else
  {
    throw std::runtime_error(std::string("KinematicsSolver: Invalid priority: ") + priority);
  }
}

Task::Task()
{
  solver = nullptr;
  priority = Soft;
  weight = 1.0;
}

void Task::set_priority_value(Priority priority_)
{
  priority = priority_;
}

void Task::set_priority(std::string priority)
{
  priority = priority_from_string(priority);
}

void Task::set_weight(double weight_)
{
  weight = weight_;
}

void Task::set_name(std::string name_)
{
  name = name_;
}

void Task::configure(std::string name_, std::string priority_, double weight_)
{
  name = name_;
  priority = priority_from_string(priority_);
  weight = weight_;
}

void Task::configure(std::string name_, Priority priority_, double weight_)
{
  name = name_;
  priority = priority_;
  weight = weight_;
}

double Task::error()
{
  return b.norm();
}
}  // namespace placo