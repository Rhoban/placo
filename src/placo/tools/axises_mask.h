#pragma once

#include <vector>
#include <string>

struct AxisesMask
{
  AxisesMask();

  void set_axises(std::string axises);

  std::vector<int> indices;
};