#include "spatial_index.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace hgvd
{
  namespace
  {
    int coordinate(Cell p, int axis) { return axis == 0 ? p.x : p.y; }
    double distance2(Cell a, Cell b)
    {
      const double x = double(a.x) - b.x, y = double(a.y) - b.y;
      return x * x + y * y;
    }
  } // namespace

  void SpatialIndex::reset(std::vector<Cell> points)
  {
    points_ = std::move(points);
    order_.resize(points_.size());
    std::iota(order_.begin(), order_.end(), 0);
    arrange(0, order_.size(), 0);
  }

  void SpatialIndex::arrange(std::size_t begin, std::size_t end, int axis)
  {
    if (end - begin < 2)
    {
      return;
    }
    const std::size_t mid = begin + (end - begin) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                     [&](auto a, auto b)
                     {
                       const int first = coordinate(points_[a], axis), second = coordinate(points_[b], axis);
                       return first == second ? a < b : first < second;
                     });
    arrange(begin, mid, 1 - axis);
    arrange(mid + 1, end, 1 - axis);
  }

  std::vector<SpatialIndex::Neighbor> SpatialIndex::nearest(Cell p, std::size_t count) const
  {
    std::vector<Neighbor> result;
    if (count)
    {
      result.reserve(std::min(count, points_.size()));
      query(p, 0, order_.size(), 0, std::numeric_limits<double>::infinity(), count, result);
    }
    return result;
  }

  std::vector<SpatialIndex::Neighbor> SpatialIndex::within(Cell p, double radius2) const
  {
    std::vector<Neighbor> result;
    query(p, 0, order_.size(), 0, radius2, 0, result);
    return result;
  }

  void SpatialIndex::query(Cell p, std::size_t begin, std::size_t end, int axis,
                           double radius2, std::size_t count, std::vector<Neighbor> &result) const
  {
    if (begin == end)
    {
      return;
    }
    const std::size_t mid = begin + (end - begin) / 2;
    const auto id = order_[mid];
    const double split = double(coordinate(p, axis)) - coordinate(points_[id], axis);
    const auto candidate = std::make_pair(distance2(p, points_[id]), id);
    if (candidate.first <= radius2)
    {
      if (!count)
      {
        result.push_back(candidate);
      }
      else
      {
        auto position = std::lower_bound(result.begin(), result.end(), candidate);
        if (position != result.end() || result.size() < count)
        {
          result.insert(position, candidate);
          if (result.size() > count)
          {
            result.pop_back();
          }
        }
      }
    }
    const bool left = split < 0;
    auto search = [&](bool use_left)
    {
      query(p, use_left ? begin : mid + 1, use_left ? mid : end,
            1 - axis, radius2, count, result);
    };
    search(left);
    const double limit = count && result.size() == count ? result.back().first : radius2;
    if (split * split <= limit)
    {
      search(!left);
    }
  }
} // namespace hgvd
