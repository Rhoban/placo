#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

struct AxisesMask
{
  AxisesMask();

  /**
   * @brief Sets the axises to be masked (kept), for example "xy"
   * @param axises axises to be kept
   * @param local true if the axises should be masked in the local frame (false by default)
   */
  void set_axises(std::string axises, bool local = false);

  /**
   * @brief Apply the masking to a given matrix
   * @param M the matrix to be masked (3xn)
   */
  Eigen::MatrixXd apply(Eigen::MatrixXd M);

  /**
   * @brief Transformation from local frame to the world
   */
  Eigen::Matrix3d R_local_world;

  std::vector<int> indices;
  bool local;
};