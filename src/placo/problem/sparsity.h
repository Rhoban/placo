#pragma once

#include <vector>
#include <Eigen/Dense>

namespace placo::problem
{
/**
 * @brief Internal helper to check the column sparsity of a matrix
 */
class Sparsity
{
public:
  /**
   * @brief An interval is a range of columns that are not sparse
   */
  struct Interval
  {
    Interval();
    Interval(int start, int end);

    /**
     * @brief Start of interval
     */
    int start = 0;

    /**
     * @brief End of interval
     */
    int end = 0;

    /**
     * @brief Does this interval contain the given column?
     * @param i column
     * @return true if the column is in the interval
     */
    bool contains(int i);
  };

  /**
   * @brief Adds an interval to the sparsity, this will compute the union of intervals
   * @param start interval start
   * @param end interval end
   */
  void add_interval(int start, int end);

  /**
   * @brief Print intervals
   */
  void print_intervals();

  /**
   * @brief Intervals of non-sparse columns
   */
  std::vector<Interval> intervals;

  /**
   * @brief Combines two sparsity objects
   * @param other other sparsity
   * @return union of the two sparsity objects
   */
  Sparsity operator+(const Sparsity& other) const;

  /**
   * @brief Helper to detect columns sparsity
   * @param M given matrix
   * @return sparsity
   */
  static Sparsity detect_columns_sparsity(const Eigen::MatrixXd M);
};
}  // namespace placo::problem