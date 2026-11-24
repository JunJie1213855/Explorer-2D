#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace hgvd
{
  struct Cell
  {
    int x, y;
    bool operator==(Cell other) const { return x == other.x && y == other.y; }
    bool operator!=(Cell other) const { return !(*this == other); }
    bool operator<(Cell other) const { return x == other.x ? y < other.y : x < other.x; }
  };

  // Deterministic two-dimensional KD tree. Query distances are squared grid units.
  class SpatialIndex
  {
  public:
    using Neighbor = std::pair<double, std::size_t>;
    void reset(std::vector<Cell> points);
    std::vector<Neighbor> nearest(Cell p, std::size_t count) const;
    std::vector<Neighbor> within(Cell p, double radius2) const;

  private:
    void arrange(std::size_t begin, std::size_t end, int axis);
    void query(Cell p, std::size_t begin, std::size_t end, int axis, double radius2,
               std::size_t count, std::vector<Neighbor> &result) const;
    std::vector<Cell> points_;
    std::vector<std::size_t> order_;
  };
} // namespace hgvd
