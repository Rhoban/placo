#pragma once

#include "placo/problem/problem.h"
#include "placo/tools/prioritized.h"

namespace placo::dynamics
{
class DynamicsSolver;
class Constraint : public tools::Prioritized
{
public:
  /**
   * @brief Reference to the dynamics solver
   */
  DynamicsSolver* solver = nullptr;

  /**
   * @brief true if this object memory is in the solver (it will be deleted by the solver)
   */
  bool solver_memory = false;

  /**
   * @brief Allows the specific constraint implementation to be added to the problem
   * @param problem problem
   * @param tau expression for tau
   */
  virtual void add_constraint(problem::Problem& problem, problem::Expression& tau) = 0;

protected:
  /**
   * @brief Buffers for compact Jacobians and their columns (their memory is reused)
   */
  Eigen::MatrixXd J_a, J_b, dJ_a, dJ_b;
  std::vector<int> columns, columns_buffer;
};
}  // namespace placo::dynamics