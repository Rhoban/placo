#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace placo
{
/**
 * @brief Used to mask some task axises
 *
 * See the \ref set_axises method
 *
 * ```cpp
 * // Will only keep the x and y axises
 * someTask.mask.set_axises("xy");
 *
 * // Will only keep the x and y axises, but in the local frame
 * someTask.mask.set_axises("xy", true);
 * ```
 */
struct AxisesMask
{
  AxisesMask();

  /**
   * @brief Sets the axises to be masked (kept), for example "xy"
   *
   * @param axises axises to be kept
   * @param local true if the axises should be masked in the local frame (false by default). Note that some tasks
   * (like relative ones) have only local representation, and this flag will have no effect on them.
   */
  void set_axises(std::string axises, bool local = false);

  /**
   * @brief Apply the masking to a given matrix
   * @param M the matrix to be masked (3xn)
   */
  Eigen::MatrixXd apply(Eigen::MatrixXd M);

  /**
   * @brief Transformation from local frame to the world used with the local flag
   */
  Eigen::Matrix3d R_local_world;

  /**
   * @brief Indices that should be kept, see \ref set_axises
   */
  std::vector<int> indices;

  /**
   * @brief whether to express the masking in local frame, see \ref set_axises
   */
  bool local;
};
}  // namespace placo