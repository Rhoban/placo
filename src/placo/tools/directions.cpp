#include "directions.h"

namespace placo::tools
{
Eigen::MatrixXd directions_2d(int n)
{
  Eigen::MatrixXd directions(n, 2);

  for (int i = 0; i < n; i++)
  {
    double angle = 2 * M_PI * i / n;
    directions(i, 0) = cos(angle);
    directions(i, 1) = sin(angle);
  }

  return directions;
}

Eigen::MatrixXd directions_3d(int n, double epsilon)
{
  Eigen::MatrixXd directions(n, 3);

  // Golden ratio
  double phi = (1 + sqrt(5)) / 2;

  for (int i = 0; i < n; i++)
  {
    double x = fmod(i / phi, 1);
    double y = (i + epsilon) / (n - 1 + 2 * epsilon);

    double alpha = 2 * M_PI * x;
    double beta = acos(1 - 2 * y);

    directions(i, 0) = sin(beta) * cos(alpha);
    directions(i, 1) = sin(beta) * sin(alpha);
    directions(i, 2) = cos(beta);
  }

  return directions;
}

}  // namespace placo::tools