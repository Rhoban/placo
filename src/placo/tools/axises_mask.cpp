#include "placo/tools/axises_mask.h"
#include <stdexcept>

namespace placo::tools
{
AxisesMask::AxisesMask()
{
  indices = { 0, 1, 2 };
  frame = ReferenceFrame::TaskFrame;
  R_local_world = Eigen::Matrix3d::Identity();
  R_custom_world = Eigen::Matrix3d::Identity();
}

void AxisesMask::set_axises(std::string axises, AxisesMask::ReferenceFrame frame_)
{
  indices.clear();
  frame = frame_;

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

void AxisesMask::set_axises(std::string axises, std::string frame_)
{
  if (frame_ == "task" || frame_ == "world")
  {
    set_axises(axises, ReferenceFrame::TaskFrame);
  }
  else if (frame_ == "local")
  {
    set_axises(axises, ReferenceFrame::LocalFrame);
  }
  else if (frame_ == "custom")
  {
    set_axises(axises, ReferenceFrame::CustomFrame);
  }
  else
  {
    throw std::runtime_error("Invalid frame: " + frame_);
  }
}

Eigen::MatrixXd AxisesMask::apply(Eigen::MatrixXd M)
{
  Eigen::MatrixXd M_masked;

  if (frame == ReferenceFrame::CustomFrame)
  {
    M_masked = R_custom_world * M;
  }
  else if (frame == ReferenceFrame::LocalFrame)
  {
    M_masked = R_local_world * M;
  }
  else
  {
    M_masked = M;
  }

  return M_masked(indices, Eigen::all);
}

void AxisesMask::apply(const Eigen::Ref<const Eigen::MatrixXd>& M, Eigen::Ref<Eigen::MatrixXd> out) const
{
  for (int k = 0; k < (int)indices.size(); k++)
  {
    if (frame == ReferenceFrame::CustomFrame)
    {
      out.row(k).noalias() = R_custom_world.row(indices[k]) * M;
    }
    else if (frame == ReferenceFrame::LocalFrame)
    {
      out.row(k).noalias() = R_local_world.row(indices[k]) * M;
    }
    else
    {
      out.row(k) = M.row(indices[k]);
    }
  }
}

int AxisesMask::rows() const
{
  return indices.size();
}
}  // namespace placo::tools