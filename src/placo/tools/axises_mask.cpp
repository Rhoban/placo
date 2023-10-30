#include "placo/tools/axises_mask.h"
#include <stdexcept>

AxisesMask::AxisesMask()
{
  indices = { 0, 1, 2 };
  local = false;
  R_local_world = Eigen::Matrix3d::Identity();
}

void AxisesMask::set_axises(std::string axises, bool local_)
{
  indices.clear();
  local = local_;

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

Eigen::MatrixXd AxisesMask::apply(Eigen::MatrixXd M)
{
  Eigen::MatrixXd M_masked;

  if (local)
  {
    M_masked = R_local_world * M;
  }
  else
  {
    M_masked = M;
  }

  return M_masked(indices, Eigen::all);
}