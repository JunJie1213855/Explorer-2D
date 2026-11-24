# Hierarchical_GVD_exploration

## ROS 2 Humble

当前 ROS 2 入口为 `hgvd_exploration_ros2/hgvd_explorer`。`hgvd/` 是独立的 Python 算法包，仅依赖 NumPy 和 SciPy。ROS 2 节点直接使用 `hgvd.explore.Planner`，在同一进程内传递采样和拓扑结果。

- `hgvd/gvd.py` 生成分层采样和 GVD 单元。
- `hgvd/utils.py` 提供地图降噪和带缓存的已知空闲连通性。
- `hgvd/mean_shift3.py`、`hgvd/topology.py` 和 `hgvd/Bidirectional_a_star.py` 负责聚类、拓扑图和双向 A* 补边。
- `hgvd/explore.py` 组装规划流程并计算候选视点代价。`hgvd_exploration_ros2/explorer_node.py` 负责 OccupancyGrid、TF、Nav2 动作和 RViz。

原 ROS 1 脚本保存在 `hgvd/legacy_ros1/`，用作算法对照。

在 Explorer-2D 工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/hgvd-exploration --ignore-src -r -y
colcon build --symlink-install --packages-select hgvd_exploration_ros2
source install/setup.bash
ros2 launch hgvd_exploration_ros2 explore.launch.py use_sim_time:=true robot_base_frame:=base_link
```

用 Gazebo 时，先启动 TurtleBot3、SLAM Toolbox 和 Nav2，各节点使用仿真时钟。最后在同一个 ROS 域内运行上面的 HGVD 启动命令。下文的 `benchmark.launch.py` 会在独立 ROS 域启动二维矩阵模拟器，适用于独立测试。默认订阅 `/map`（`nav_msgs/msg/OccupancyGrid`），使用地图消息中的坐标系，向 `navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）发送可达的空闲格中心。启动参数可调整地图话题、基座坐标系、动作名和仿真时钟。使用实体机器人的系统时，省略 `use_sim_time:=true`。专用 RViz 配置见下文。

规划器随 TF 更新局部采样窗口，全局使用更疏的采样网格。GVD 单元按半径去重，Mean Shift 只使用满足连通性约束的邻居。拓扑图优先用可直达的采样点建边，必要时用双向 A* 在已知空闲区域补边。目标是 GVD 单元内关联的可达前沿视点。缺少足够的障碍物或 GVD 样本时，通过占用栅格前沿继续探索。降噪产生的空闲格不会用于判断导航可达性。

可用 ROS 参数调整 `local_radius_m`（5.0）、`local_sample_stride`（2 格）、`global_sample_stride`（8 格）、`cluster_bandwidth_m`（2.0）、`frontier_spacing_m`（0.6）、`obstacle_clearance_m`（0.0）、`min_goal_distance_m`（0.35）及 `goal_switch_margin_m`（1.0）。默认障碍净距为零，方便运行点机器人基准测试，可根据实体机器人和 Nav2 的安全距离配置调整。目标拒绝、失败或超时后会暂时跳过对应目标。若自定义了 Nav2 的 `xy_goal_tolerance`，将 `min_goal_distance_m` 设为略大于该值，避免导航器在原地宣布到达。

### 探索结束后返回起点

```bash
ros2 launch hgvd_exploration_ros2 explore.launch.py use_sim_time:=true return_to_start:=true rviz:=true
```

`return_to_start` 默认关闭。启用后，节点记录首次获得有效地图和 TF 时的坐标及航向。在没有正在执行的探索目标、且连续 `completion_hold_s` 秒（默认 5 秒）没有可达前沿时，HGVD 将该位姿交给 Nav2。地图的栅格原点或尺寸变化时，仍使用记录的地图坐标。

距离过滤或失败目标冷却造成的暂时空闲不会触发返航。返航目标失败后按 `failed_goal_retry_s` 间隔重试，最多派发 3 次。预览模式不派发返航目标。

### RViz 可视化

使用 `rviz:=true` 随探索节点打开专用视图。默认值为 `false`。

```bash
ros2 launch hgvd_exploration_ros2 explore.launch.py use_sim_time:=true rviz:=true
```

如果想在车静止时查看局部拓扑，可以同时启用预览模式：

```bash
ros2 launch hgvd_exploration_ros2 explore.launch.py use_sim_time:=true rviz:=true navigation_enabled:=false
```

预览模式需要地图和机器人 TF，可与 Gazebo、SLAM Toolbox 一起使用。它会计算并发布拓扑和候选视点。开始自主探索前先结束预览节点，再按正常方式启动探索。

如果探索节点已经运行，可用 `ros2 launch hgvd_exploration_ros2 rviz.launch.py` 单独打开视图。

| 图层 | 显示内容 |
| --- | --- |
| SLAM Map / TurtleBot3 | 占用地图和由 TF 定位的机器人模型 |
| HGVD Topology | 青色采样点、薄荷绿半径圆、紫色聚类中心、青蓝色直连边、粉紫色 A* 拓扑连边、橙色候选视点 |
| Active HGVD Goal | 亮绿色箭头，表示 Nav2 已接受的 HGVD 目标及朝向 |
| Nav2 Global Plan | 绿色曲线，表示 Nav2 发布的全局路径 |

可在 RViz 的 **HGVD Topology → Namespaces** 中独立开关图层。A* 拓扑连边只连接图中的两个端点，机器人实际行驶路线看 **Nav2 Global Plan**。已有 RViz 窗口时，也可以打开软件包里的 `rviz/hgvd.rviz`，或单独添加 `/hgvd_explorer/topology`（MarkerArray）和 `/hgvd_explorer/goal_marker`（Marker）。

### 分阶段计时

使用新文件名记录耗时。若文件已存在，请重新生成文件名。

```bash
HGVD_CSV="$PWD/results/hgvd_timing/python_$(date +%Y%m%d_%H%M%S_%N).csv"
ros2 launch hgvd_exploration_ros2 explore.launch.py \
  use_sim_time:=true rviz:=true timing_csv:="$HGVD_CSV"

# 停止节点后绘图，需要 Matplotlib 和 NumPy
python3 src/hgvd-exploration/scripts/plot_timings.py "$HGVD_CSV"
```

原始图、CSV 和统一设置下的复测命令见 [Python / C++ 耗时对比](../../docs/hgvd_timing/README.md)。

### 二维基准测试

无需 Gazebo、SLAM 或实际 Nav2 即可测试 ROS 接口：

```bash
colcon build --symlink-install --packages-select hgvd_exploration_ros2 explore_grid_benchmark
source install/setup.bash
ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=hgvd scenario:=rooms
ros2 launch explore_grid_benchmark suite.launch.py algorithms:=hgvd scenarios:=open,rooms,corridor,clutter
```

如果在 WSL 的默认 DDS 配置下收不到同机测试器的地图，可在已安装 Cyclone DDS 的情况下将测试进程限制在本机。在上述任一测试命令前加上以下环境变量：

```bash
ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=hgvd scenario:=rooms
```

## Upstream

**Paper has been submitted to IEEE Transactions on Automation Science and Engineering (TASE). We will release our code upon acceptance.** 🚀🚀🚀

**Abstract**: Topological maps are more suitable than metric maps for robotic exploration tasks. However, real-time updating of accurate and detail-rich environmental topological maps remains a challenge. This paper presents a topological map updating method based on the Generalized Voronoi Diagram (GVD). First, the newly observed areas are denoised to avoid low-efficiency GVD nodes misleading the topological structure. Subsequently, a multi-granularity hierarchical GVD generation method is designed to control the sampling granularity at both global and local levels. This not only ensures the accuracy of the topological structure but also enhances the ability to capture detail features, reduces the probability of path backtracking, and ensures no overlap between GVDs through the maintenance of a coverage map, thereby improving GVD utilization efficiency. Second, a node clustering method with connectivity constraints and a connectivity method based on a switching mechanism are designed to avoid the generation of unreachable nodes and erroneous nodes caused by obstacle attraction. A special cache structure is used to store all connectivity information, thereby improving exploration efficiency. Finally, to address the issue of frontiers misjudgment caused by obstacles within the scope of GVD units, a frontiers extraction method based on morphological dilation is designed to effectively ensure the reachability of frontiers. On this basis, a lightweight cost function is used to assess and switch to the next viewpoint in real time. This allows the robot to quickly adjust its strategy when signs of path backtracking appear, thereby escaping the predicament and increasing exploration flexibility. The performance of system for exploration task is verified through ablation experiments and comparative tests with SOTA methods.

<img width="995" height="657" alt="image" src="https://github.com/user-attachments/assets/7fccb318-b35f-40b6-b162-eeae1b6db9be" />
Fig.1 Graphic abstract

\



<img width="1441" height="439" alt="image" src="https://github.com/user-attachments/assets/9b6b33e8-4647-4dcc-8f30-da147dd19f4f" />

Fig.2 Pipeline
