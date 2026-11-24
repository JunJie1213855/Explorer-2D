#include "planner.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
  using hgvd::Cell;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Point = geometry_msgs::msg::Point;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using Action = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  constexpr double tau = 6.28318530717958647692;

  template <typename Rotation>
  double yaw_of(const Rotation &q)
  {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  Point point(double x, double y, double z = 0.0)
  {
    Point value;
    value.x = x;
    value.y = y;
    value.z = z;
    return value;
  }

  struct Geometry
  {
    std::string frame;
    int width, height;
    double resolution, x, y, yaw;

    bool operator==(const Geometry &other) const
    {
      return std::tie(frame, width, height, resolution, x, y, yaw) ==
             std::tie(other.frame, other.width, other.height, other.resolution,
                      other.x, other.y, other.yaw);
    }

    std::pair<double, double> grid_position(double px, double py) const
    {
      const double dx = px - x, dy = py - y;
      return {(std::cos(yaw) * dx + std::sin(yaw) * dy) / resolution - 0.5,
              (-std::sin(yaw) * dx + std::cos(yaw) * dy) / resolution - 0.5};
    }

    std::optional<Cell> grid_cell(double gx, double gy) const
    {
      if (!(gx >= -0.5 && gy >= -0.5 && gx < width - 0.5 && gy < height - 0.5))
      {
        return std::nullopt;
      }
      return Cell{int(std::floor(gx + 0.5)), int(std::floor(gy + 0.5))};
    }

    Point center(Cell cell, double z = 0.0) const
    {
      const double dx = (cell.x + 0.5) * resolution, dy = (cell.y + 0.5) * resolution;
      return point(x + std::cos(yaw) * dx - std::sin(yaw) * dy,
                   y + std::sin(yaw) * dx + std::cos(yaw) * dy, z);
    }
  };

  struct ActiveGoal
  {
    std::size_t id;
    Cell cell;
    Pose pose;
    double sent_at;
    std::size_t map_revision;
    bool returning;
    Handle::SharedPtr handle;
    bool canceling = false;
  };
} // namespace

class Explorer : public rclcpp::Node
{
public:
  Explorer()
      : Node("hgvd_explorer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    robot_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
    const auto action = declare_parameter<std::string>("navigate_action", "navigate_to_pose");
    const bool navigation = declare_parameter("navigation_enabled", true);
    return_enabled_ = declare_parameter("return_to_start", false);
    const bool visualize = declare_parameter("visualize", true);
    period_ = numeric_parameter("planning_period_s", 0.5);
    start_search_ = numeric_parameter("unknown_start_search_m", 0.25, true);
    min_distance_ = numeric_parameter("min_goal_distance_m", 0.35, true);
    switch_margin_ = numeric_parameter("goal_switch_margin_m", 1.0, true);
    goal_timeout_ = numeric_parameter("goal_timeout_s", 120.0);
    retry_delay_ = numeric_parameter("failed_goal_retry_s", 30.0);
    threshold_ = declare_parameter("occupied_threshold", 65);
    if (threshold_ < 1 || threshold_ > 100)
    {
      throw std::invalid_argument("occupied_threshold must be in [1, 100]");
    }

    hgvd::Config cfg;
    cfg.local_radius = numeric_parameter("local_radius_m", 5.0);
    cfg.bandwidth = numeric_parameter("cluster_bandwidth_m", 2.0);
    cfg.clearance = numeric_parameter("obstacle_clearance_m", 0.0, true);
    cfg.spacing = numeric_parameter("frontier_spacing_m", 0.6);
    cfg.local_stride = declare_parameter("local_sample_stride", 2);
    cfg.global_stride = declare_parameter("global_sample_stride", 8);
    configure_timing(declare_parameter<std::string>("timing_csv", ""));
    planner_ = std::make_unique<hgvd::Planner>(cfg, timing_);

    if (navigation)
    {
      nav_ = rclcpp_action::create_client<Action>(this, action);
    }
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    auto live = rclcpp::QoS(1).best_effort().durability_volatile();
    auto callback = [this](OccupancyGrid::ConstSharedPtr msg)
    { on_map(*msg); };
    map_latched_ = create_subscription<OccupancyGrid>(map_topic, latched, callback);
    map_live_ = create_subscription<OccupancyGrid>(map_topic, live, callback);
    if (visualize)
    {
      markers_ = create_publisher<MarkerArray>("~/topology", latched);
      goal_marker_ = create_publisher<Marker>("~/goal_marker", latched);
    }
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(period_),
                                  [this]()
                                  {
                                    hgvd::Measure time(timing_, "planning_tick_total");
                                    plan();
                                  });
  }

private:
  double numeric_parameter(const std::string &name, double value, bool allow_zero = false)
  {
    const auto parameter = declare_parameter(name, value);
    if (!std::isfinite(parameter) || parameter < 0 || (!allow_zero && parameter == 0))
    {
      throw std::invalid_argument("Invalid " + name);
    }
    return parameter;
  }

  void configure_timing(const std::string &filename)
  {
    if (filename.empty())
    {
      return;
    }
    const std::filesystem::path path(filename);
    if (path.has_parent_path())
    {
      std::filesystem::create_directories(path.parent_path());
    }
    timing_file_.reset(std::fopen(path.c_str(), "wx"));
    if (!timing_file_)
    {
      throw std::system_error(errno, std::generic_category(), "Opening timing CSV " + filename);
    }
    std::fputs("elapsed_s,stage,duration_ms\n", timing_file_.get());
    const auto epoch = hgvd::Clock::now();
    timing_ = [this, epoch](const char *stage, auto start, auto duration)
    {
      std::fprintf(timing_file_.get(), "%.9f,%s,%.6f\n",
                   std::chrono::duration<double>(start - epoch).count(), stage,
                   std::chrono::duration<double, std::milli>(duration).count());
      std::fflush(timing_file_.get());
    };
    RCLCPP_INFO(get_logger(), "Recording HGVD timings to %s", filename.c_str());
  }

  void on_map(const OccupancyGrid &msg)
  {
    if (msg.info.width > uint32_t(std::numeric_limits<int>::max()) ||
        msg.info.height > uint32_t(std::numeric_limits<int>::max()))
    {
      return;
    }
    const auto &origin = msg.info.origin;
    Geometry next{msg.header.frame_id, int(msg.info.width), int(msg.info.height),
                  msg.info.resolution, origin.position.x, origin.position.y, yaw_of(origin.orientation)};
    if (next.frame.empty() || !next.width || !next.height ||
        !std::isfinite(next.resolution) || next.resolution <= 0 ||
        !std::isfinite(next.x) || !std::isfinite(next.y) || !std::isfinite(next.yaw) ||
        msg.data.size() != std::size_t(next.width) * next.height ||
        msg.data.size() > std::size_t(std::numeric_limits<int>::max()))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid occupancy grid");
      return;
    }
    std::vector<int8_t> grid;
    {
      hgvd::Measure time(timing_, "map_decode");
      grid.reserve(msg.data.size());
      for (const auto value : msg.data)
      {
        grid.push_back(value >= threshold_ ? 100 : (value == 0 ? 0 : -1));
      }
    }
    const bool changed = !geometry_ || !(*geometry_ == next);
    if (!changed && grid == planner_->grid())
    {
      return;
    }
    planner_->update_map(next.width, next.height, next.resolution, std::move(grid), changed);
    if (changed)
    {
      ++map_revision_;
      excluded_.clear();
    }
    geometry_ = std::move(next);
  }

  void plan()
  {
    if (return_done_)
    {
      return;
    }
    if (!geometry_)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for occupancy grid");
      return;
    }
    const double now = get_clock()->now().seconds();
    if (active_)
    {
      if (active_->handle)
      {
        publish_goal(&active_->pose);
      }
      if (active_->pose.header.frame_id != geometry_->frame ||
          (!active_->returning && active_->map_revision != map_revision_))
      {
        cancel_goal();
        return;
      }
      if (now - active_->sent_at > goal_timeout_)
      {
        if (!active_->returning && !active_->canceling)
        {
          excluded_[active_->cell] = now + retry_delay_;
        }
        cancel_goal();
        return;
      }
      if (active_->returning)
      {
        return;
      }
    }

    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(geometry_->frame, robot_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &error)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for TF: %s", error.what());
      return;
    }
    const double x = transform.transform.translation.x, y = transform.transform.translation.y;
    const double yaw = yaw_of(transform.transform.rotation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
    {
      return;
    }
    const auto [gx, gy] = geometry_->grid_position(x, y);
    const auto pose_cell = geometry_->grid_cell(gx, gy);
    const auto robot = pose_cell ? planner_->start_cell(*pose_cell, gx, gy, start_search_) : std::nullopt;
    if (!robot || !planner_->update(*robot))
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Robot pose is outside known traversable space");
      return;
    }

    const bool anchored = *pose_cell != *robot;
    if (anchored)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                           "Robot pose is in unknown space; using nearby free planning cell (%d, %d)",
                           robot->x, robot->y);
    }

    if (nav_ && return_enabled_)
    {
      if (!anchored && !start_pose_)
      {
        start_pose_ = make_pose(point(x, y), yaw);
      }
      if (exploration_complete_)
      {
        try_return(now);
        return;
      }
    }

    for (auto it = excluded_.begin(); it != excluded_.end();)
    {
      if (it->second <= now)
      {
        it = excluded_.erase(it);
      }
      else
      {
        ++it;
      }
    }
    if (active_)
    {
      if (!planner_->reachable(active_->cell))
      {
        cancel_goal();
        return;
      }
      if (!active_->handle || active_->canceling)
      {
        return;
      }
    }
    std::vector<Cell> targets;
    {
      hgvd::Measure time(timing_, "viewpoint_selection");
      targets = planner_->viewpoints(*robot, gx, gy, min_distance_, excluded_);
    }
    publish_topology(targets);
    if (targets.empty())
    {
      if (!active_)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                             "No reachable HGVD viewpoints meet the goal filters");
        if (nav_ && return_enabled_)
        {
          if (start_pose_)
          {
            exploration_complete_ = true;
            RCLCPP_INFO(get_logger(), "No eligible HGVD goal; returning to the start pose");
            try_return(now);
          }
          else
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "Cannot return until a valid start pose has been recorded");
          }
        }
      }
      return;
    }
    if (!nav_)
    {
      return;
    }
    Cell choice;
    double best, present = 0;
    {
      hgvd::Measure time(timing_, "goal_scoring");
      auto scoring_targets = targets;
      if (active_ && std::find(targets.begin(), targets.end(), active_->cell) == targets.end())
      {
        scoring_targets.push_back(active_->cell);
      }
      const auto scores = planner_->scores(*robot, yaw - geometry_->yaw, scoring_targets);
      choice = *std::min_element(targets.begin(), targets.end(), [&](Cell a, Cell b)
                                 { return std::make_pair(scores.at(a), a) < std::make_pair(scores.at(b), b); });
      best = scores.at(choice);
      if (active_)
      {
        present = scores.at(active_->cell);
      }
    }
    if (!std::isfinite(best))
    {
      return;
    }
    if (active_)
    {
      if (choice != active_->cell && present - best > switch_margin_)
      {
        cancel_goal();
      }
    }
    else if (nav_->action_server_is_ready())
    {
      const auto heading = std::atan2(choice.y - gy, choice.x - gx) + geometry_->yaw;
      send(choice, make_pose(geometry_->center(choice), heading), now, false);
    }
    else
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for NavigateToPose");
    }
  }

  Pose make_pose(const Point &position, double yaw) const
  {
    Pose pose;
    pose.header.frame_id = geometry_->frame;
    pose.pose.position = position;
    pose.pose.orientation.z = std::sin(yaw / 2);
    pose.pose.orientation.w = std::cos(yaw / 2);
    return pose;
  }

  void try_return(double now)
  {
    publish_topology({});
    if (now < return_retry_at_ || !nav_->action_server_is_ready())
    {
      return;
    }
    const auto &start = *start_pose_;
    if (start.header.frame_id != geometry_->frame)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Waiting for the original map frame");
      return;
    }
    const auto [gx, gy] = geometry_->grid_position(start.pose.position.x, start.pose.position.y);
    const auto cell = geometry_->grid_cell(gx, gy);
    if (!cell || !planner_->reachable(*cell))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Start pose is not in reachable space");
      return;
    }
    ++return_attempts_;
    send(*cell, start, now, true);
  }

  void send(Cell cell, Pose pose, double now, bool returning)
  {
    pose.header.stamp = get_clock()->now();
    const auto id = ++goal_id_;
    active_ = ActiveGoal{id, cell, pose, now, map_revision_, returning, nullptr};
    Action::Goal goal;
    goal.pose = std::move(pose);
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this, id](Handle::SharedPtr handle)
    {
      if (!active_ || active_->id != id)
      {
        return;
      }
      if (!handle)
      {
        finish(rclcpp_action::ResultCode::UNKNOWN, "rejected");
        return;
      }
      active_->handle = std::move(handle);
      publish_goal(&active_->pose);
      RCLCPP_INFO(get_logger(), "%s (%.3f, %.3f)", active_->returning ? "Returning to start" : "Navigating to HGVD viewpoint",
                  active_->pose.pose.position.x, active_->pose.pose.position.y);
    };
    options.result_callback = [this, id](const Handle::WrappedResult &result)
    {
      if (active_ && active_->id == id)
      {
        finish(result.code, "Nav2 result");
      }
    };
    try
    {
      hgvd::Measure time(timing_, "nav2_goal_dispatch");
      nav_->async_send_goal(goal, options);
    }
    catch (const std::exception &error)
    {
      finish(rclcpp_action::ResultCode::UNKNOWN, error.what());
    }
  }

  void cancel_goal()
  {
    if (!active_->handle || active_->canceling)
    {
      return;
    }
    try
    {
      const auto id = active_->id;
      active_->canceling = true;
      nav_->async_cancel_goal(active_->handle, [this, id](auto response)
                              {
        if (active_ && active_->id == id && response->goals_canceling.empty()) {
          active_->canceling = false;
        } });
    }
    catch (const std::exception &error)
    {
      active_->canceling = false;
      RCLCPP_WARN(get_logger(), "Could not cancel HGVD goal: %s", error.what());
    }
  }

  void finish(rclcpp_action::ResultCode result, const char *detail)
  {
    const double now = get_clock()->now().seconds();
    const bool success = result == rclcpp_action::ResultCode::SUCCEEDED;
    if (active_->returning)
    {
      if (success)
      {
        return_done_ = true;
        RCLCPP_INFO(get_logger(), "Returned to the start pose");
      }
      else if (return_attempts_ >= 3)
      {
        return_done_ = true;
        RCLCPP_WARN(get_logger(), "Could not return to the start after 3 attempts: %s", detail);
      }
      else
      {
        return_retry_at_ = now + retry_delay_;
        RCLCPP_WARN(get_logger(), "Return failed; will retry: %s", detail);
      }
    }
    else if (active_->map_revision == map_revision_)
    {
      if (success)
      {
        excluded_[active_->cell] = now + 2.0;
      }
      else if (result != rclcpp_action::ResultCode::CANCELED || !active_->canceling)
      {
        excluded_[active_->cell] = now + retry_delay_;
        RCLCPP_WARN(get_logger(), "HGVD goal failed: %s (%d)", detail, int(result));
      }
    }
    active_.reset();
    publish_goal(nullptr);
  }

  Marker marker(const std::string &ns, int kind, double scale, float red, float green,
                float blue, float alpha = 1.0) const
  {
    Marker msg;
    msg.header.frame_id = geometry_->frame;
    msg.ns = ns;
    msg.type = kind;
    msg.pose.orientation.w = 1;
    msg.scale.x = msg.scale.y = scale;
    msg.color.r = red;
    msg.color.g = green;
    msg.color.b = blue;
    msg.color.a = alpha;
    msg.lifetime = rclcpp::Duration::from_seconds(std::max(5.0, 3 * period_));
    return msg;
  }

  void publish_goal(const Pose *pose)
  {
    if (!goal_marker_)
    {
      return;
    }
    auto msg = marker("active_goal", Marker::ARROW, 0.12, 0.36, 1, 0.22);
    if (pose)
    {
      msg.header.frame_id = pose->header.frame_id;
      msg.pose = pose->pose;
      msg.pose.position.z = 0.17;
      msg.scale.x = 0.34;
      msg.scale.z = 0.12;
    }
    else
    {
      msg.action = Marker::DELETE;
    }
    goal_marker_->publish(msg);
  }

  void publish_topology(const std::vector<Cell> &targets)
  {
    if (!markers_)
    {
      return;
    }
    hgvd::Measure time(timing_, "rviz_markers");
    if (marker_revision_ != planner_->topology_revision())
    {
      marker_revision_ = planner_->topology_revision();
      auto rings = marker("gvd_radii", Marker::LINE_LIST, 0.018, 0.20, 0.78, 0.66, 0.55);
      auto samples = marker("samples", Marker::POINTS, 0.08, 0.16, 0.85, 1);
      auto centers = marker("centers", Marker::POINTS, 0.14, 0.68, 0.36, 1);
      auto edges = marker("edges", Marker::LINE_LIST, 0.022, 0.15, 0.66, 0.84);
      auto bridges = marker("astar_links", Marker::LINE_LIST, 0.022, 0.87, 0.40, 0.81);
      for (const auto &sample : planner_->samples())
      {
        const auto center = geometry_->center(sample.cell, 0.09);
        samples.points.push_back(center);
        const double radius = sample.radius * geometry_->resolution;
        for (int i = 0; i < 32; ++i)
        {
          for (int j : {i, i + 1})
          {
            const double a = tau * j / 32;
            rings.points.push_back(point(center.x + radius * std::cos(a),
                                         center.y + radius * std::sin(a), 0.04));
          }
        }
      }
      for (auto p : planner_->centers())
      {
        centers.points.push_back(geometry_->center(p, 0.12));
      }
      for (std::size_t i = 0; i < planner_->graph().size(); ++i)
      {
        for (const auto &e : planner_->graph()[i])
        {
          if (i >= e.to)
          {
            continue;
          }
          auto &layer = e.direct ? edges : bridges;
          layer.points.push_back(geometry_->center(planner_->vertices()[i], 0.06));
          layer.points.push_back(geometry_->center(planner_->vertices()[e.to], 0.06));
        }
      }
      static_markers_.markers.clear();
      for (auto *layer : {&rings, &samples, &centers, &edges, &bridges})
      {
        if (layer->points.empty())
        {
          layer->action = Marker::DELETE;
        }
        static_markers_.markers.push_back(std::move(*layer));
      }
    }
    MarkerArray message = static_markers_;
    auto viewpoints = marker("viewpoints", Marker::POINTS, 0.12, 1.0, 0.65, 0.12);
    for (auto p : targets)
    {
      viewpoints.points.push_back(geometry_->center(p, 0.11));
    }
    if (viewpoints.points.empty())
    {
      viewpoints.action = Marker::DELETE;
    }
    message.markers.push_back(std::move(viewpoints));
    markers_->publish(message);
  }

  std::string robot_frame_;
  double period_, min_distance_, switch_margin_, goal_timeout_, retry_delay_;
  double start_search_;
  int threshold_;
  bool return_enabled_;
  std::unique_ptr<std::FILE, decltype(&std::fclose)> timing_file_{nullptr, &std::fclose};
  hgvd::Timing timing_;
  std::unique_ptr<hgvd::Planner> planner_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_latched_, map_live_;
  rclcpp_action::Client<Action>::SharedPtr nav_;
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<Marker>::SharedPtr goal_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<Geometry> geometry_;
  std::optional<ActiveGoal> active_;
  std::optional<Pose> start_pose_;
  std::map<Cell, double> excluded_;
  std::size_t map_revision_ = 0, marker_revision_ = 0, goal_id_ = 0;
  MarkerArray static_markers_;
  bool exploration_complete_ = false, return_done_ = false;
  int return_attempts_ = 0;
  double return_retry_at_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try
  {
    rclcpp::spin(std::make_shared<Explorer>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("hgvd_explorer"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
