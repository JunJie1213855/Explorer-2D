# HGVD Exploration C++

`hgvd_exploration` 是独立的 ROS 2 Humble 包。C++ 核心处理地图降噪、分层 GVD 采样、连通性约束 Mean Shift、拓扑建图、双向 A*、前沿提取和评分。C++ 节点通过 OccupancyGrid 和 TF 接收输入，通过 Nav2 的 NavigateToPose 派发探索与返航目标。

实现沿用 Python 版的核心策略。障碍物距离场在地图更新时计算，直连缓存可跨同一地图的局部重建复用，Mean Shift 缓存同一中心的迭代结果，图搜索按需执行。距离场使用 OpenCV 浮点结果，等距候选使用确定性顺序，个别样本或目标的排序可能与 Python 版不同。

在 Explorer-2D 根目录构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hgvd_exploration explore_grid_benchmark \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

Gazebo、SLAM 和 Nav2 就绪后运行：

```bash
ros2 launch hgvd_exploration explore.launch.py \
  use_sim_time:=true rviz:=true return_to_start:=true
```

`rviz` 和 `return_to_start` 默认关闭。`navigation_enabled:=false` 只预览图层。启用返航后，当前导航目标结束且候选点通过筛选后为空时，立即尝试返回本轮记录的起点。距离过滤和失败冷却都可能使候选点列表为空。Python 和 C++ 节点使用相同的目标动作和图层话题，同一个机器人一次使用一个版本。

当 SLAM 暂时将机器人所在格标为未知时，`unknown_start_search_m`（默认 0.25 米）允许从附近已知空闲格锚定拓扑。目标距离仍按真实 TF 位姿计算，导航仍由 Nav2 执行。设为 0 可关闭此行为。

记录各阶段的壁钟耗时并沿用现有绘图脚本。测量纯规划时可将 `rviz` 和 `visualize` 都设为 `false`，这样也会停止生成 Marker：

```bash
HGVD_CSV="$PWD/results/hgvd_timing/cpp_$(date +%Y%m%d_%H%M%S).csv"
ros2 launch hgvd_exploration explore.launch.py \
  use_sim_time:=true rviz:=false visualize:=false timing_csv:="$HGVD_CSV"

# 退出节点后绘图，需要 python3-matplotlib
python3 src/hgvd-exploration/scripts/plot_timings.py "$HGVD_CSV"
```

使用矩阵测试器比较两个版本：

```bash
ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch explore_grid_benchmark suite.launch.py \
  scenarios:=rooms,corridor algorithms:=hgvd,hgvd_cpp
```

运行对比前也需要构建 `hgvd_exploration_ros2`。可用 `benchmark.launch.py algorithm:=hgvd_cpp` 只运行 C++ 版。
