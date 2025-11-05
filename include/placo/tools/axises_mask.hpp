#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace placo::tools
{
/**
 * @brief Used to mask some task axises
 *
 * See the \ref set_axises method
 *
 * ```cpp
 * // Will only keep the x and y axises
 * someTask.mask.set_axises("xy", "task");
 *
 * // Will only keep the x and y axises, but in the local frame
 * someTask.mask.set_axises("xy", "local");
 *
 * // Will only keep the x and y axises, in a custom frame
 * someTask.mask.set_axises("xy", "custom");
 * someTask.mask.R_custom_world = R_table_world;
 * ```
 */
struct AxisesMask
{
  /**
   * @brief The reference frame where the masking is done
   */
  enum ReferenceFrame
  {
    /**
     * @brief Use the (usually world) frame provided by the task
     */
    TaskFrame = 0,
    /**
     * @brief Use the local frame provided by the task in \ref R_local_world
     */
    LocalFrame = 1,
    /**
     * @brief Use the custom frame provided by the user in \ref R_custom_world
     */
    CustomFrame = 2
  };

  AxisesMask();

  /**
   * @brief Sets the axises to be masked (kept), for example "xy"
   *
   * @param axises axises to be kept
   * @param frame_ the reference frame where the masking is done (task, local or custom)
   */
  void set_axises(std::string axises, ReferenceFrame frame = TaskFrame);

  /**
   * @brief Sets the axises to be masked (kept), for example "xy"
   *
   * @param axises axises to be kept
   * @param frame the reference frame where the masking is done (task, local or custom)
   */
  void set_axises(std::string axises, std::string frame);

  /**
   * @brief Apply the masking to a given matrix
   * @param M the matrix to be masked (3xn)
   */
  Eigen::MatrixXd apply(Eigen::MatrixXd M);

  /**
   * @brief Rotation from world to local frame (provided by task)
   */
  Eigen::Matrix3d R_local_world;

  /**
   * @brief Rotation from world to custom rotation (provided by the user)
   */
  Eigen::Matrix3d R_custom_world;

  /**
   * @brief Indices that should be kept, see \ref set_axises
   */
  std::vector<int> indices;

  /**
   * @brief The reference frame where the masking is done, see \ref set_axises
   */
  ReferenceFrame frame;
};
}  // namespace placo::tools