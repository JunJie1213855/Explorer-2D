# HGVD 探索

`hgvd_exploration` 是 Explorer-2D 的 ROS 2 Humble C++ 包，源码位于 `src/hgvd-exploration/`。它从占用地图提取可达前沿，通过分层 GVD 采样、连通性约束 Mean Shift 和拓扑图选择视点，必要时用双向 A* 补连图分量。HGVD 派发目标，Nav2 负责路径规划、避障和运动控制。

## 构建

在 Explorer-2D 根目录执行。HGVD 核心使用 OpenCV。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/hgvd-exploration --ignore-src -r -y
colcon build --symlink-install --packages-select hgvd_exploration \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

## Gazebo 或实机启动

先启动机器人或 Gazebo、SLAM 和 Nav2。HGVD 默认订阅 `/map`（`nav_msgs/msg/OccupancyGrid`），查询地图坐标系到 `base_link` 的 TF，并向 `navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）派发目标。

```bash
ros2 launch hgvd_exploration explore.launch.py use_sim_time:=true rviz:=true
```

实机使用系统时钟时，省略 `use_sim_time:=true`。以下参数可根据环境调整：

| 启动参数 | 默认值 | 用途 |
| --- | --- | --- |
| `map_topic` | `/map` | 占用地图话题 |
| `robot_base_frame` | `base_link` | 机器人基座 TF 坐标系 |
| `navigate_action` | `navigate_to_pose` | Nav2 动作名 |
| `navigation_enabled` | `true` | 派发导航目标。设为 `false` 可静止预览 |
| `return_to_start` | `false` | 没有活跃目标且过滤后没有候选点时，立即尝试返回本轮记录的起点 |
| `min_goal_distance_m` | `0.35` | 目标到机器人真实 TF 位姿的最小距离，单位为米 |
| `unknown_start_search_m` | `0.25` | 机器人所在格为未知时，搜索附近已知空闲规划锚点的半径，单位为米。设为 0 可关闭 |
| `rviz` | `false` | 启动 HGVD 专用 RViz |
| `visualize` | `true` | 发布拓扑和目标图层 |
| `timing_csv` | 空 | 启用分阶段计时并写入指定的新文件 |

使用 `return_to_start:=true` 启用返航。距离过滤、失败目标的冷却期和前沿耗尽都可能使候选列表为空。重启节点会重新记录起点。

## RViz 预览

```bash
ros2 launch hgvd_exploration explore.launch.py \
  use_sim_time:=true rviz:=true navigation_enabled:=false
```

RViz 显示 SLAM 地图、GVD 采样和半径圆、聚类中心、拓扑边、候选视点和当前导航目标。从 **HGVD Topology → Namespaces** 可切换图层。拓扑话题为 `/hgvd_explorer/topology`，目标话题为 `/hgvd_explorer/goal_marker`。探索节点已运行时，可以用 `ros2 launch hgvd_exploration rviz.launch.py` 单独打开视图。

## 阶段耗时

```bash
HGVD_CSV="$PWD/results/hgvd_timing/hgvd_$(date +%Y%m%d_%H%M%S_%N).csv"
ros2 launch hgvd_exploration explore.launch.py \
  use_sim_time:=true rviz:=false visualize:=false timing_csv:="$HGVD_CSV"
```

每次启动都要使用新路径，已有 CSV 会保留。文件列为 `elapsed_s,stage,duration_ms`，记录地图处理、GVD 采样、聚类、构图、视点评分等阶段的实际耗时。需要测量 Marker 生成开销时，可保持 `visualize:=true`。`planning_tick_total` 已包含该轮内各阶段的耗时。

## 二维矩阵测试

在工作区根目录构建 HGVD 和测试器：

```bash
colcon build --symlink-install --packages-select hgvd_exploration explore_grid_benchmark \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=hgvd scenario:=rooms
```

可用 `suite.launch.py scenarios:=rooms,corridor algorithms:=hgvd` 依次运行多个场景。测试器提供自己的地图、TF 和导航动作服务器。场景和指标说明见[二维矩阵测试器](../explore_grid_benchmark/README.md)。

## 源码结构

- `include/planner.h`、`src/planner.cpp` 定义地图处理、GVD、拓扑图和视点评分。
- `include/spatial_index.h`、`src/spatial_index.cpp` 实现二维 KD 树，供采样点和图节点的近邻查询使用。
- `src/explorer.cpp` 连接 ROS 2 地图、TF、Nav2、RViz 和 CSV 计时。
- `test/planner_test.cpp` 包含核心算法测试。构建后可运行 `ctest --test-dir build/hgvd_exploration --output-on-failure`。
