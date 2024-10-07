#pragma once

#include <vector>
#include <Eigen/Dense>

namespace placo::tools
{
/**
 * @brief Generates a set of directions in 2D
 * @param n the number of directions
 * @return matrix of size n x 2
 */
Eigen::MatrixXd directions_2d(int n);

/**
 * @brief Generates a set of directions in 3D, using Fibonacci lattice
 * @param n the number of directions
 * @param epsilon the epsilon parameter for the Fibonacci lattice
 * @return matrix of size n x 3
 */
Eigen::MatrixXd directions_3d(int n, double epsilon = 0.5);
}  // namespace placo::tools