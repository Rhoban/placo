#pragma once

#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/tools/utils.h"

namespace placo::tools
{
/**
 * @brief Represents an object (like a task or a constraint) that is prioritized.
 */
class Prioritized
{
public:
  /**
   * @brief Represents the task priority, which can be hard, soft or scaled
   */
  enum Priority
  {
    /**
     * @brief Task that **has** to be enforced, will result in equality constraint,
     */
    Hard = 0,
    /**
     * @brief Task that can be unenforced, will result in an objective function (can be weighted)
     */
    Soft = 1,
    /**
     * @brief This is similar to ``Hard``, but with a scaling factor (that is also a decision variable)
     */
    Scaled = 2
  };

  Prioritized();
  virtual ~Prioritized();

  /**
   * @brief Object name
   */
  std::string name;

  /**
   * @brief Configures the object
   * @param name task name
   * @param priority task priority (hard, soft or scaled)
   * @param weight task weight
   */
  void configure(std::string name, std::string priority = "soft", double weight = 1.0);

  /**
   * @brief Configures the object
   * @param name task name
   * @param priority task priority (hard, soft or scaled)
   * @param weight task weight
   */
  void configure(std::string name, Priority priority = Soft, double weight = 1.0);

  /**
   * @brief Object priority
   */
  Priority priority;

  /**
   * @brief Object priority
   * @return string representing the task priority
   */
  std::string priority_name();

  /**
   * @brief The oject weight, used for soft tasks only
   */
  double weight;
};
}  // namespace placo::tools