#include "placo/control/axises_mask.h"
#include <stdexcept>

AxisesMask::AxisesMask()
{
  indices = { 0, 1, 2 };
}

void AxisesMask::set_axises(std::string axises)
{
  indices.clear();

  for (char& c : axises)
  {
    c = std::tolower(c);

    if (c == 'x')
    {
      indices.push_back(0);
    }
    else if (c == 'y')
    {
      indices.push_back(1);
    }
    else if (c == 'z')
    {
      indices.push_back(2);
    }
    else
    {
      throw std::runtime_error("Invalid axis: " + std::to_string(c));
    }
  }
}