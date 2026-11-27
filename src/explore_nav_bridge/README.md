# explore_nav_bridge

把 Explore-2D 系列探索器（`hgvd_exploration`、`rrt-explore-ros2`、`mrtsp_exploration_ros2`）
使用的 Nav2 `NavigateToPose` action 接到 topic 式导航栈上。

适用于不使用 Nav2 的规划/控制组合，例如：

- `astar_global_node`：订阅 `/goal_pose`（`PoseStamped`）+ `/map`，发布 `/initial_path`（`Path`）
- `neupan_cpp_node`：订阅 `/initial_path` + `/scan`，发布 `/cmd_vel`、`/neupan_arrive`（`Bool`）

## 行为

- 提供 `/navigate_to_pose` action server（`nav2_msgs/action/NavigateToPose`）。
- 收到 goal 后把 `goal.pose` 发布到 `/goal_pose`。
- `/neupan_arrive` 为 10Hz 周期 `Bool`：跟踪中为 `false`，到达后持续 `true`。
  桥接节点在发出 goal 后**先看到一次 `false`（确认新路径被接受）**，再看到 `true`
  才回 `SUCCEEDED`，避免把上一个 goal 遗留的 `true` 当成到达。
- 取消 goal 时：向 `/initial_path` 发布空路径让 NeuPAN 停车（`clear_path_on_cancel:=true`），
  并短时发布零 `/cmd_vel`。若新 goal 立刻到来，停车过程会提前退出，不会清掉新路径。
- 超时（默认 120s）回 `ABORTED`，并打印 arrival 信号是否曾出现。

## 运行

```bash
colcon build --packages-select explore_nav_bridge --symlink-install
source install/setup.bash
ros2 launch explore_nav_bridge nav_bridge.launch.py use_sim_time:=true
```

然后在另一个终端启动探索器，并把仿真时间打开：

```bash
ros2 launch hgvd_exploration explore.launch.py use_sim_time:=true
```

参数（launch 中均可覆盖）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `navigate_action` | `navigate_to_pose` | action server 名，需与探索器一致 |
| `goal_pose_topic` | `/goal_pose` | 发给全局规划器的目标位姿 |
| `arrive_topic` | `/neupan_arrive` | 到达信号 |
| `path_topic` | `/initial_path` | 取消时用于清空参考路径 |
| `cmd_vel_topic` | `/cmd_vel` | 取消时用于刹车 |
| `map_frame` / `base_frame` | `map` / `base_link` | feedback 中当前位姿的 TF |
| `goal_timeout_s` | `120.0` | 单目标超时 |
| `feedback_period_s` | `0.3` | feedback 周期 |
| `cancel_stop_duration_s` | `0.5` | 取消时零速度持续时长 |
| `clear_path_on_cancel` | `true` | 取消时是否发空路径让 NeuPAN 停车 |
| `unaccepted_goal_warn_s` | `5.0` | 新目标迟迟未被接受的告警阈值 |

## 已知限制

- 依赖 NeuPAN 的 `false -> true` 到达语义；换成其它局部规划器时需替换
  `arrive_topic` 或修改到达判定。
- `base_frame` 在 Gazebo/TurtleBot3 上通常是 `base_footprint`，按你的 TF 树改。
