#include "planner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace hgvd
{
  namespace
  {
    constexpr std::array<Cell, 4> steps{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double tau = 6.28318530717958647692;

    Cell add(Cell a, Cell b) { return {a.x + b.x, a.y + b.y}; }
    double squared(Cell a, Cell b)
    {
      const double x = double(a.x) - b.x, y = double(a.y) - b.y;
      return x * x + y * y;
    }
    int manhattan(Cell a, Cell b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

  } // namespace

  Planner::Planner(Config config, Timing timing)
      : config_(config), timing_(std::move(timing))
  {
    if (config_.local_stride < 1 || config_.global_stride <= config_.local_stride ||
        !std::isfinite(config_.local_radius) || config_.local_radius <= 0 ||
        !std::isfinite(config_.bandwidth) || config_.bandwidth <= 0 ||
        !std::isfinite(config_.spacing) || config_.spacing <= 0 ||
        !std::isfinite(config_.clearance) || config_.clearance < 0)
    {
      throw std::invalid_argument("Invalid HGVD configuration");
    }
  }

  bool Planner::inside(Cell p) const
  {
    return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_;
  }

  bool Planner::reachable(Cell p) const
  {
    return component_ > 0 && inside(p) && components_[index(p)] == component_;
  }

  std::optional<Cell> Planner::start_cell(Cell robot, double gx, double gy, double search_m) const
  {
    if (!inside(robot))
    {
      return std::nullopt;
    }
    const auto id = index(robot);
    if (components_[id])
    {
      return robot;
    }
    // Recover from a SLAM observation gap at the robot, while keeping
    // obstacle cells and out-of-map poses ineligible for recovery.
    if (grid_[id] != -1 || search_m <= 0)
    {
      return std::nullopt;
    }

    std::vector<Cell> pending{robot};
    std::set<Cell> visited{robot};
    std::optional<Cell> best;
    double nearest = inf;
    for (std::size_t j = 0; j < pending.size(); ++j)
    {
      for (const auto step : steps)
      {
        const auto next = add(pending[j], step);
        if (!inside(next) || !visited.insert(next).second)
        {
          continue;
        }
        const double distance = std::hypot(next.x - gx, next.y - gy) * resolution_;
        if (distance > search_m)
        {
          continue;
        }
        const auto i = index(next);
        if (components_[i] && (!best || std::make_pair(distance, next) <
                                            std::make_pair(nearest, *best)))
        {
          nearest = distance;
          best = next;
        }
        else if (grid_[i] == -1)
        {
          pending.push_back(next);
        }
      }
    }
    return best;
  }

  void Planner::update_map(int width, int height, double resolution, std::vector<int8_t> grid, bool reset)
  {
    if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0 ||
        grid.size() != std::size_t(width) * height ||
        grid.size() > std::size_t(std::numeric_limits<int>::max()))
    {
      throw std::invalid_argument("Invalid occupancy grid");
    }
    const bool resized = reset || width != width_ || height != height_;
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    cv::Mat free(height, width, CV_8U), obstacles(height, width, CV_8U);
    obstacle_rows_.assign(height, {});
    std::size_t count = 0;
    for (std::size_t i = 0; i < grid.size(); ++i)
    {
      free.ptr<uint8_t>()[i] = grid[i] == 0;
      obstacles.ptr<uint8_t>()[i] = grid[i] == 100;
      if (grid[i] == 100)
      {
        const auto p = cell(i);
        obstacle_rows_[p.y].push_back(p.x);
        ++count;
      }
    }
    {
      Measure time(timing_, "map_denoise");
      cv::Mat support, change(height, width, CV_8U, cv::Scalar(1));
      cv::boxFilter(free, support, CV_8U, {5, 5}, {-1, -1}, false, cv::BORDER_CONSTANT);
      if (resized)
      {
        clean_ = grid;
      }
      else
      {
        for (std::size_t i = 0; i < grid.size(); ++i)
        {
          change.ptr<uint8_t>()[i] = grid[i] != grid_[i];
        }
        cv::dilate(change, change, cv::Mat::ones(5, 5, CV_8U));
      }
      for (std::size_t i = 0; i < grid.size(); ++i)
      {
        if (change.ptr<uint8_t>()[i])
        {
          clean_[i] = grid[i] == -1 && support.ptr<uint8_t>()[i] >= 3 ? 0 : grid[i];
        }
      }
    }
    grid_ = std::move(grid);
    {
      Measure time(timing_, "map_connectivity");
      cv::Mat nearby;
      cv::dilate(obstacles, nearby, cv::Mat::ones(5, 5, CV_8U));
      wall_.assign(nearby.ptr<uint8_t>(), nearby.ptr<uint8_t>() + grid_.size());
      euclidean_.assign(grid_.size(), float(inf));
      manhattan_.clear();
      if (count)
      {
        cv::Mat background = 1 - obstacles, distance;
        cv::distanceTransform(background, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        std::copy_n(distance.ptr<float>(), grid_.size(), euclidean_.begin());
        if (count >= 2)
        {
          cv::distanceTransform(background, distance, cv::DIST_L1, 3);
          manhattan_.assign(distance.ptr<float>(), distance.ptr<float>() + grid_.size());
        }
      }
      components_.assign(grid_.size(), 0);
      auto free_cell = [&](int i)
      {
        return grid_[i] == 0 && euclidean_[i] * resolution + 1e-6 >= config_.clearance;
      };
      std::vector<int> queue;
      int component = 0;
      for (int start = 0; start < int(grid_.size()); ++start)
      {
        if (components_[start] || !free_cell(start))
        {
          continue;
        }
        components_[start] = ++component;
        queue.assign(1, start);
        for (std::size_t j = 0; j < queue.size(); ++j)
        {
          for (auto step : steps)
          {
            const auto p = add(cell(queue[j]), step);
            if (!inside(p))
            {
              continue;
            }
            const int i = index(p);
            if (!components_[i] && free_cell(i))
            {
              components_[i] = component;
              queue.push_back(i);
            }
          }
        }
      }
    }
    sample_origin_.reset();
    component_ = 0;
    direct_cache_.clear();
  }

  bool Planner::update(Cell robot)
  {
    if (!inside(robot) || !components_[index(robot)])
    {
      return false;
    }
    const auto component = components_[index(robot)];
    if (component == component_ && sample_origin_ &&
        squared(robot, *sample_origin_) < double(config_.local_stride) * config_.local_stride)
    {
      return true;
    }
    if (component != component_)
    {
      direct_cache_.clear();
    }
    component_ = component;
    sample_origin_ = robot;
    {
      Measure time(timing_, "gvd_sampling");
      sample(robot);
      std::vector<Cell> points;
      points.reserve(samples_.size());
      for (const auto &sample : samples_)
      {
        points.push_back(sample.cell);
      }
      sample_index_.reset(std::move(points));
    }
    {
      Measure time(timing_, "mean_shift");
      cluster();
    }
    {
      Measure time(timing_, "topology_graph");
      connect_graph();
    }
    {
      Measure time(timing_, "frontier_detection");
      detect_frontiers();
    }
    ++topology_revision_;
    return true;
  }

  void Planner::sample(Cell robot)
  {
    samples_.clear();
    if (manhattan_.empty())
    {
      return;
    }
    const int radius = int(std::min(std::ceil(config_.local_radius / resolution_),
                                    std::ceil(std::hypot(width_, height_))));
    std::vector<Sample> candidates;
    for (bool local : {true, false})
    {
      const int stride = local ? config_.local_stride : config_.global_stride;
      const int y0 = local ? std::max(0, robot.y - radius) : 0;
      const int y1 = local ? std::min(height_, robot.y + radius + 1) : height_;
      const int x0 = local ? std::max(0, robot.x - radius) : 0;
      const int x1 = local ? std::min(width_, robot.x + radius + 1) : width_;
      for (int y = y0; y < y1; y += stride)
      {
        for (int x = x0; x < x1; x += stride)
        {
          const Cell p{x, y};
          if ((squared(robot, p) <= double(radius) * radius) != local || !reachable(p))
          {
            continue;
          }
          const int i = index(p);
          if (euclidean_[i] < 2.0)
          {
            continue;
          }
          const int range = int(manhattan_[i]) + 2;
          int min_x = width_, max_x = -1, min_y = height_, max_y = -1;
          for (int row = std::max(0, y - range); row <= std::min(height_ - 1, y + range); ++row)
          {
            const int dx = range - std::abs(row - y);
            const auto &sites = obstacle_rows_[row];
            const auto first = std::lower_bound(sites.begin(), sites.end(), x - dx);
            const auto last = std::upper_bound(first, sites.end(), x + dx);
            if (first == last)
            {
              continue;
            }
            min_x = std::min(min_x, *first);
            max_x = std::max(max_x, *(last - 1));
            min_y = std::min(min_y, row);
            max_y = row;
            if (max_x - min_x > 5 || max_y - min_y > 5)
            {
              candidates.push_back({p, euclidean_[i]});
              break;
            }
          }
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b)
              { return a.radius != b.radius ? a.radius > b.radius : a.cell < b.cell; });
    std::vector<uint8_t> covered(grid_.size());
    std::vector<int> visited(grid_.size(), -1);
    std::vector<int> queue;
    for (const auto &cand : candidates)
    {
      const int start = index(cand.cell);
      if (covered[start])
      {
        continue;
      }
      const int id = samples_.size();
      samples_.push_back(cand);
      queue.assign(1, start);
      visited[start] = id;
      const double radius2 = cand.radius * cand.radius + 1e-5;
      for (std::size_t j = 0; j < queue.size(); ++j)
      {
        covered[queue[j]] = 1;
        for (const auto step : steps)
        {
          const auto p = add(cell(queue[j]), step);
          if (!inside(p) || squared(p, cand.cell) > radius2)
          {
            continue;
          }
          const int i = index(p);
          if (visited[i] != id && clean_[i] == 0)
          {
            visited[i] = id;
            queue.push_back(i);
          }
        }
      }
    }
  }

  bool Planner::direct(Cell a, Cell b) const
  {
    if (b < a)
    {
      std::swap(a, b);
    }
    const auto key = std::make_pair(a, b);
    if (const auto hit = direct_cache_.find(key); hit != direct_cache_.end())
    {
      return hit->second;
    }
    const auto visible = [&]()
    {
      if (!reachable(a) || !reachable(b))
      {
        return false;
      }
      const int dx = std::abs(b.x - a.x), dy = -std::abs(b.y - a.y);
      const int sx = a.x < b.x ? 1 : -1, sy = a.y < b.y ? 1 : -1;
      int error = dx + dy;
      while (a != b)
      {
        const Cell previous = a;
        const int twice = 2 * error;
        if (twice >= dy)
        {
          a.x += sx;
          error += dy;
        }
        if (twice <= dx)
        {
          a.y += sy;
          error += dx;
        }
        if (!reachable(a))
        {
          return false;
        }
        if (a.x != previous.x && a.y != previous.y &&
            (!reachable({a.x, previous.y}) || !reachable({previous.x, a.y})))
        {
          return false;
        }
      }
      return true;
    }();
    if (direct_cache_.size() >= 65536)
    {
      direct_cache_.clear();
    }
    direct_cache_.emplace(key, visible);
    return visible;
  }

  void Planner::cluster()
  {
    const double bandwidth2 = std::pow(config_.bandwidth / resolution_, 2);
    struct Step
    {
      Cell next;
      int neighbors;
    };
    std::map<Cell, Step> shifts;
    std::map<Cell, int> modes;
    for (const auto &sample : samples_)
    {
      Cell center = sample.cell;
      int intensity = 0;
      for (int iteration = 0; iteration < 50; ++iteration)
      {
        auto it = shifts.find(center);
        if (it == shifts.end())
        {
          double x = 0, y = 0;
          int count = 0;
          for (const auto &[distance, i] : sample_index_.within(center, bandwidth2))
          {
            const auto other = samples_[i].cell;
            if (direct(center, other))
            {
              x += other.x;
              y += other.y;
              ++count;
            }
          }
          Cell next = center;
          if (count)
          {
            const Cell mean{int(std::nearbyint(x / count)), int(std::nearbyint(y / count))};
            if (direct(center, mean))
            {
              next = mean;
            }
          }
          it = shifts.emplace(center, Step{next, count}).first;
        }
        intensity = it->second.neighbors;
        if (!intensity || it->second.next == center || iteration == 49)
        {
          break;
        }
        center = it->second.next;
      }
      if (intensity)
      {
        modes[center] = std::max(modes[center], intensity);
      }
    }
    std::vector<std::pair<Cell, int>> ordered(modes.begin(), modes.end());
    std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b)
              { return a.second != b.second ? a.second > b.second : a.first < b.first; });
    centers_.clear();
    for (const auto &[p, count] : ordered)
    {
      if (std::none_of(centers_.begin(), centers_.end(), [&](Cell center)
                       { return squared(center, p) <= bandwidth2 && direct(center, p); }))
      {
        centers_.push_back(p);
      }
    }
  }

  std::vector<int> Planner::distances(Cell start) const
  {
    std::vector<int> distance(grid_.size(), -1), queue{index(start)};
    distance[queue.front()] = 0;
    for (std::size_t j = 0; j < queue.size(); ++j)
    {
      for (auto step : steps)
      {
        const auto p = add(cell(queue[j]), step);
        if (!reachable(p))
        {
          continue;
        }
        const int i = index(p);
        if (distance[i] < 0)
        {
          distance[i] = distance[queue[j]] + 1;
          queue.push_back(i);
        }
      }
    }
    return distance;
  }

  double Planner::bidirectional_astar(Cell start, Cell goal) const
  {
    if (start == goal)
    {
      return 0;
    }
    using Entry = std::tuple<int64_t, int, int>;
    using Queue = std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>>;
    std::array<Queue, 2> open;
    std::array<std::map<int, int>, 2> cost;
    auto potential = [&](Cell p)
    { return int64_t(manhattan(p, goal)) - manhattan(p, start); };
    open[0].emplace(potential(start), 0, index(start));
    open[1].emplace(-potential(goal), 0, index(goal));
    cost[0][index(start)] = cost[1][index(goal)] = 0;
    double best = inf;
    while (!open[0].empty() && !open[1].empty())
    {
      if (std::get<0>(open[0].top()) + std::get<0>(open[1].top()) >= 2 * best)
      {
        break;
      }
      const int side = open[0].top() <= open[1].top() ? 0 : 1;
      const auto [key, length, id] = open[side].top();
      open[side].pop();
      if (cost[side].at(id) != length)
      {
        continue;
      }
      for (const auto step : steps)
      {
        const auto p = add(cell(id), step);
        if (!reachable(p))
        {
          continue;
        }
        const int i = index(p);
        const auto it = cost[side].find(i);
        if (it != cost[side].end() && it->second <= length + 1)
        {
          continue;
        }
        cost[side][i] = length + 1;
        if (const auto other = cost[1 - side].find(i); other != cost[1 - side].end())
        {
          best = std::min(best, double(length + 1) + other->second);
        }
        open[side].emplace(2LL * (length + 1) + (side ? -1 : 1) * potential(p), length + 1, i);
      }
    }
    return best;
  }

  void Planner::connect_graph()
  {
    vertices_ = centers_;
    std::set<Cell> known(vertices_.begin(), vertices_.end());
    for (const auto &sample : samples_)
    {
      if (known.insert(sample.cell).second)
      {
        vertices_.push_back(sample.cell);
      }
    }
    graph_.assign(vertices_.size(), {});
    graph_index_.reset(vertices_);
    if (vertices_.size() < 2)
    {
      return;
    }
    auto add_edge = [&](std::size_t a, std::size_t b, double weight, bool visible)
    {
      graph_[a].push_back({b, weight, visible});
      graph_[b].push_back({a, weight, visible});
    };
    {
      Measure time(timing_, "topology_direct_edges");
      std::set<std::pair<std::size_t, std::size_t>> examined;
      for (std::size_t a = 0; a < vertices_.size(); ++a)
      {
        for (const auto &[distance, b] : graph_index_.nearest(vertices_[a], 11))
        {
          if (a != b && examined.insert(std::minmax(a, b)).second &&
              direct(vertices_[a], vertices_[b]))
          {
            add_edge(a, b, std::sqrt(distance), true);
          }
        }
      }
    }
    std::vector<bool> joined(vertices_.size());
    std::vector<std::size_t> members{0};
    joined[0] = true;
    std::size_t scanned = 0;
    while (true)
    {
      for (; scanned < members.size(); ++scanned)
      {
        for (const auto &e : graph_[members[scanned]])
        {
          if (!joined[e.to])
          {
            joined[e.to] = true;
            members.push_back(e.to);
          }
        }
      }
      if (members.size() == vertices_.size())
      {
        break;
      }
      Measure time(timing_, "topology_astar_bridge");
      std::vector<Cell> connected;
      connected.reserve(members.size());
      for (const auto a : members)
      {
        connected.push_back(vertices_[a]);
      }
      SpatialIndex connected_index;
      connected_index.reset(std::move(connected));
      std::vector<std::tuple<double, std::size_t, std::size_t>> candidates;
      for (std::size_t b = 0; b < vertices_.size(); ++b)
      {
        if (joined[b])
        {
          continue;
        }
        const auto closest = connected_index.nearest(vertices_[b], 1).front();
        candidates.emplace_back(closest.first, b, members[closest.second]);
      }
      const auto count = std::min<std::size_t>(10, candidates.size());
      std::partial_sort(candidates.begin(), candidates.begin() + count, candidates.end());
      double best = inf;
      std::size_t first = 0, second = 0;
      for (std::size_t i = 0; i < count; ++i)
      {
        const auto [lower, b, a] = candidates[i];
        if (lower >= best * best)
        {
          break;
        }
        const double length = bidirectional_astar(vertices_[a], vertices_[b]);
        if (length < best)
        {
          best = length;
          first = a;
          second = b;
        }
      }
      if (!std::isfinite(best))
      {
        break;
      }
      add_edge(first, second, best, false);
      joined[second] = true;
      members.push_back(second);
    }
  }

  void Planner::detect_frontiers()
  {
    frontiers_.clear();
    for (int i = 0; i < int(grid_.size()); ++i)
    {
      if (components_[i] != component_)
      {
        continue;
      }
      const auto p = cell(i);
      if (std::any_of(steps.begin(), steps.end(), [&](Cell step)
                      {
      const auto q = add(p, step);
      return inside(q) && grid_[index(q)] == -1; }))
      {
        frontiers_.push_back(p);
      }
    }
  }

  std::vector<Cell> Planner::viewpoints(Cell robot, double x, double y, double minimum_m,
                                        const std::map<Cell, double> &excluded) const
  {
    const int spacing = int(std::clamp(std::nearbyint(config_.spacing / resolution_),
                                       1.0, double(std::max(width_, height_))));
    using Rank = std::tuple<bool, double, double, Cell>;
    std::map<std::tuple<bool, int, int>, Rank> groups;
    for (const auto p : frontiers_)
    {
      if (p == robot || excluded.count(p) ||
          std::hypot(p.x - x, p.y - y) * resolution_ < minimum_m)
      {
        continue;
      }
      Cell reference = robot;
      auto group = std::make_tuple(false, p.x / spacing, p.y / spacing);
      for (const auto &[distance, i] : sample_index_.nearest(p, 4))
      {
        const auto &sample = samples_[i];
        if (distance <= sample.radius * sample.radius + 1e-5 && direct(sample.cell, p))
        {
          reference = sample.cell;
          group = std::make_tuple(true, int(i), 0);
          break;
        }
      }
      Rank rank{wall_[index(p)] != 0, squared(p, reference), squared(p, robot), p};
      const auto [it, inserted] = groups.emplace(group, rank);
      if (!inserted && rank < it->second)
      {
        it->second = rank;
      }
    }
    std::vector<Cell> goals;
    for (const auto &group : groups)
    {
      goals.push_back(std::get<3>(group.second));
    }
    return goals;
  }

  std::map<Cell, double> Planner::scores(Cell robot, double yaw, const std::vector<Cell> &targets) const
  {
    using Route = std::pair<double, int>;
    std::vector<Route> routes;
    std::vector<int> fallback;
    bool routes_ready = false;
    auto compute_routes = [&]()
    {
      using Entry = std::pair<Route, std::size_t>;
      std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
      routes.assign(vertices_.size(), {inf, 0});
      for (std::size_t i = 0; i < vertices_.size(); ++i)
      {
        if (direct(robot, vertices_[i]))
        {
          routes[i] = {std::sqrt(squared(robot, vertices_[i])), 0};
          queue.emplace(routes[i], i);
        }
      }
      while (!queue.empty())
      {
        const auto [cost, from] = queue.top();
        queue.pop();
        if (cost != routes[from])
        {
          continue;
        }
        for (const auto &e : graph_[from])
        {
          const Route next{cost.first + e.length, cost.second + 1};
          if (next < routes[e.to])
          {
            routes[e.to] = next;
            queue.emplace(next, e.to);
          }
        }
      }
      routes_ready = true;
    };
    const double attach2 = std::pow(config_.bandwidth / resolution_, 2);
    std::map<Cell, double> result;
    for (const auto p : targets)
    {
      const double angle = std::abs(std::remainder(std::atan2(p.y - robot.y, p.x - robot.x) - yaw, tau));
      double cost = inf;
      if (direct(robot, p))
      {
        cost = std::sqrt(squared(robot, p)) * (1 + 0.1 * angle);
      }
      else if (reachable(p))
      {
        if (!routes_ready)
        {
          compute_routes();
        }
        for (const auto &[tail2, i] : graph_index_.within(p, attach2))
        {
          const auto [length, hops] = routes[i];
          if (std::isfinite(length) && direct(vertices_[i], p))
          {
            cost = std::min(cost, length * (1 + 0.1 * (hops + 1)) + std::sqrt(tail2) * (1 + angle));
          }
        }
        if (!std::isfinite(cost))
        {
          if (fallback.empty())
          {
            fallback = distances(robot);
          }
          if (fallback[index(p)] >= 0)
          {
            cost = fallback[index(p)] * (1 + 0.1 * angle);
          }
        }
      }
      result[p] = cost * resolution_;
    }
    return result;
  }
} // namespace hgvd
