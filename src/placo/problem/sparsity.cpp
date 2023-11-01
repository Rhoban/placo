#include <iostream>
#include "placo/problem/sparsity.h"

namespace placo::problem
{
Sparsity::Interval::Interval()
{
}

Sparsity::Interval::Interval(int start, int end) : start(start), end(end)
{
}

bool Sparsity::Interval::contains(int i)
{
  return (start <= i) && (i <= end);
}

void Sparsity::add_interval(int start, int end)
{
  std::vector<Interval> old_intervals = intervals;
  intervals.clear();

  bool inserted = false;
  for (Interval old_interval : old_intervals)
  {
    if (inserted)
    {
      // We already inserted our interval, we insert the others
      intervals.push_back(old_interval);
    }
    else
    {
      if (end < old_interval.start)
      {
        // Our interval ends before the next one starts
        intervals.push_back(Interval(start, end));
        intervals.push_back(old_interval);
        inserted = true;
      }
      else if (old_interval.contains(start) && old_interval.contains(end))
      {
        // Our interval is just included in the old one
        intervals.push_back(old_interval);
        inserted = true;
      }
      else if (old_interval.contains(start))
      {
        start = old_interval.start;
      }
      else if (old_interval.contains(end))
      {
        intervals.push_back(Interval(start, old_interval.end));
        inserted = true;
      }
      else
      {
        if (start > old_interval.start)
        {
          intervals.push_back(old_interval);
        }
      }
    }
  }

  if (!inserted)
  {
    intervals.push_back(Interval(start, end));
  }
}

Sparsity Sparsity::operator+(const Sparsity& other) const
{
  Sparsity s;

  for (auto& interval : intervals)
  {
    s.add_interval(interval.start, interval.end);
  }
  for (auto& interval : other.intervals)
  {
    s.add_interval(interval.start, interval.end);
  }

  return s;
}

void Sparsity::print_intervals()
{
  std::cout << "Sparsity intervals: " << std::endl;
  for (auto interval : intervals)
  {
    std::cout << "* " << interval.start << " -> " << interval.end << std::endl;
  }
}

Sparsity Sparsity::detect_columns_sparsity(const Eigen::MatrixXd M)
{
  Sparsity sparsity;
  int last_nonzero_column = -1;

  for (int column = 0; column < M.cols(); column++)
  {
    if (M.col(column).isZero(1e-12))
    {
      if (last_nonzero_column != -1)
      {
        sparsity.add_interval(last_nonzero_column, column - 1);
        last_nonzero_column = -1;
      }
    }
    else
    {
      if (last_nonzero_column == -1)
      {
        last_nonzero_column = column;
      }
    }
  }

  if (last_nonzero_column != -1)
  {
    sparsity.add_interval(last_nonzero_column, M.cols() - 1);
  }

  return sparsity;
}
};  // namespace placo::problem