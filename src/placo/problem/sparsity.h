#pragma once

#include <vector>
#include <Eigen/Dense>

namespace placo
{
class Sparsity
{
public:
  struct Interval
  {
    Interval();
    Interval(int start, int end);

    int start = 0;
    int end = 0;

    bool contains(int i);
  };

  void add_interval(int start, int end);
  void print_intervals();
  std::vector<Interval> intervals;

  Sparsity operator+(const Sparsity& other) const;
  static Sparsity detect_columns_sparsity(const Eigen::MatrixXd M);
};
}  // namespace placo