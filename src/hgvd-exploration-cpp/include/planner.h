#pragma once

#include "spatial_index.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace hgvd
{
  using Clock = std::chrono::steady_clock;
  using Timing = std::function<void(const char *, Clock::time_point, Clock::duration)>;

  class Measure
  {
  public:
    Measure(const Timing &timing, const char *stage)
        : timing_(timing), stage_(stage), start_(timing ? Clock::now() : Clock::time_point{}) {}
    ~Measure()
    {
      if (timing_)
      {
        timing_(stage_, start_, Clock::now() - start_);
      }
    }
    Measure(const Measure &) = delete;
    Measure &operator=(const Measure &) = delete;

  private:
    const Timing &timing_;
    const char *stage_;
    Clock::time_point start_;
  };

  struct Sample
  {
    Cell cell;
    double radius;
  };
  struct Edge
  {
    std::size_t to;
    double length;
    bool direct;
  };
  struct Config
  {
    double local_radius = 5.0, bandwidth = 2.0, clearance = 0.0, spacing = 0.6;
    int local_stride = 2, global_stride = 8;
  };

  class Planner
  {
  public:
    explicit Planner(Config config, Timing timing = {});
    void update_map(int width, int height, double resolution, std::vector<int8_t> grid, bool reset = false);
    bool update(Cell robot);
    bool reachable(Cell cell) const;
    std::optional<Cell> start_cell(Cell robot, double gx, double gy, double search_m) const;
    std::vector<Cell> viewpoints(Cell robot, double x, double y, double minimum_m,
                                 const std::map<Cell, double> &excluded) const;
    std::map<Cell, double> scores(Cell robot, double yaw, const std::vector<Cell> &targets) const;

    const auto &grid() const { return grid_; }
    const auto &samples() const { return samples_; }
    const auto &centers() const { return centers_; }
    const auto &vertices() const { return vertices_; }
    const auto &graph() const { return graph_; }
    bool has_frontiers() const { return !frontiers_.empty(); }
    std::size_t topology_revision() const { return topology_revision_; }

  private:
    bool inside(Cell p) const;
    int index(Cell p) const { return p.y * width_ + p.x; }
    Cell cell(int i) const { return {i % width_, i / width_}; }
    bool direct(Cell a, Cell b) const;
    double bidirectional_astar(Cell a, Cell b) const;
    std::vector<int> distances(Cell start) const;
    void sample(Cell robot);
    void cluster();
    void connect_graph();
    void detect_frontiers();

    Config config_;
    Timing timing_;
    int width_ = 0, height_ = 0, component_ = 0;
    double resolution_ = 0.0;
    std::optional<Cell> sample_origin_;
    std::size_t topology_revision_ = 0;
    std::vector<int8_t> grid_, clean_;
    std::vector<int> components_;
    std::vector<float> euclidean_, manhattan_;
    std::vector<std::vector<int>> obstacle_rows_;
    std::vector<uint8_t> wall_;
    std::vector<Cell> frontiers_, centers_, vertices_;
    std::vector<Sample> samples_;
    std::vector<std::vector<Edge>> graph_;
    SpatialIndex sample_index_, graph_index_;
    mutable std::map<std::pair<Cell, Cell>, bool> direct_cache_;
  };
} // namespace hgvd
