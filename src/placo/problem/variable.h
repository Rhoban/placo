#pragma once

#include "placo/problem/expression.h"
#include <string>

namespace placo::problem
{
class Problem;

/**
 * @brief Represents a variable in a \ref Problem
 */
class Variable
{
public:
  /**
   * @brief Builds an expression from a variable
   * @param start start row (default: 0)
   * @param rows number of rows (default: -1, all rows)
   * @return expression
   */
  Expression expr(int start = -1, int rows = -1);

  /**
   * @brief Start offset in the \ref Problem
   */
  int k_start;

  /**
   * @brief End offset in the \ref Problem
   */
  int k_end;

  /**
   * @brief Variable size
   * @return size
   */
  int size();

  /**
   * @brief Variable value, populated by \ref Problem after a solve
   */
  Eigen::VectorXd value;

  /**
   * @brief Variable version, incremented by \ref Problem after a solve
   */
  int version = 0;

  /**
   * @brief Variable's problem
   */
  Problem* problem = nullptr;
};
};  // namespace placo::problem