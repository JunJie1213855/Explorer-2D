# Explorer-2D

基于 ROS 2 的二维移动机器人自主探索工作区，集成占用栅格前沿探索、RRT 探索、轻量前沿探索、MRTSP 探索和 HGVD 拓扑探索，以及多机器人地图合并功能，便于开展算法学习、仿真调试与探索策略对比。

前沿（Frontier）是地图中已知空闲区域与未知区域的交界。探索节点根据地图选择下一个目标，由 Nav2 完成路径规划、避障和运动控制；SLAM 持续更新地图，形成探索闭环。

```text
传感器 → SLAM 建图 → 探索节点选择目标 → Nav2 导航 → 机器人运动
                       ↑                            │
                       └──────── 地图更新 ────────────┘
```

## 项目结构

```text
Explorer-2D/
├── src/
│   ├── frontier-occ-exploration/       # 占用栅格前沿探索与历史仿真资源
│   ├── rrt-explore-ros2/              # RRT 探索
│   ├── m-explore-ros2/
│   │   ├── explore/                   # explore_lite 探索节点
│   │   ├── explore_lite_msgs/         # 探索状态消息
│   │   └── map_merge/                 # 多机器人地图合并
│   ├── mrtsp_exploration_ros2/        # 地图优化、WFD 与 MRTSP 探索
│   ├── hgvd-exploration/              # C++ HGVD 拓扑探索
│   └── explore_grid_benchmark/        # 独立二维矩阵探索测试器
├── docs/
│   └── perf_ros2_profiling.md         # perf 采样与火焰图指南
├── .gitignore
└── README.md
```

目录名与 ROS 包名并不总是一致，构建和启动时请使用下表中的 ROS 包名。

| ROS 包名 | 语言 | 功能 | 主要可执行程序 |
| --- | --- | --- | --- |
| `frontier_exploration` | C++ | 前沿检测、质心计算、障碍物膨胀和目标派发 | `frontier_planner` |
| `rrt_explore` | C++ | 随机树扩展与探索目标发现，包含多机器人相关逻辑 | `rrt` |
| `explore_lite` | C++ | 轻量前沿探索，支持暂停、恢复和可选返回起点 | `explore` |
| `explore_lite_msgs` | ROS 消息 | 定义探索状态消息 | — |
| `multirobot_map_merge` | C++ | 多机器人占用栅格地图合并 | `map_merge` |
| `mrtsp_exploration_ros2` | Python | 地图优化、WFD 前沿提取和 MRTSP 代价排序 | `mrtsp_explorer` |
| `hgvd_exploration` | C++ | 分层 GVD 采样、拓扑建图和 Nav2 探索 | `hgvd_explorer` |
| `explore_grid_benchmark` | Python | 二维矩阵世界、模拟导航与探索指标统计 | `grid_simulator` |

## 探索算法

### 占用栅格前沿探索

`frontier_exploration` 从 `/map` 中提取前沿并计算质心，通过 Nav2 的 `NavigateToPose` 动作接口派发目标，提供前沿、质心、膨胀地图和目标等可视化，以及 `get_centroids` 服务。

该模块由 ROS 1 版本移植而来，目录中仍保留旧版 launch、URDF、Gazebo 场景和模型。旧版 ROS 1 资源不能直接作为 ROS 2 启动入口。

### RRT 探索

`rrt_explore` 在地图上随机采样、扩展树并发现探索目标，结合代价地图向 Nav2 发送目标，可显示随机树与前沿点，并通过话题控制探索启停。

当前单机器人配置要求节点使用 `robot1` 这类命名空间，同时连接根命名空间下的地图、TF 和 Nav2 接口。多机器人运行还需要配置合并地图、各机器人坐标系及导航接口。

### 轻量前沿探索与地图合并

`explore_lite` 提供轻量前沿探索，支持通过 `explore/resume` 暂停和恢复，以及通过 `return_to_init` 参数控制探索结束后是否返回起点。

`multirobot_map_merge` 提供地图合并，并包含已知、未知初始位姿的配置与多 TurtleBot3 仿真启动文件。实际使用需要核对 SLAM 地图、命名空间及坐标变换的兼容性，详见子项目文档。

### MRTSP 探索

`mrtsp_exploration_ros2` 包含以下处理流程：

1. 接收占用栅格地图、全局与局部代价地图。
2. 使用双边滤波与膨胀生成探索决策地图。
3. 使用 WFD 风格的搜索提取、聚类前沿。
4. 综合距离、前沿规模与运动约束构造 MRTSP 代价矩阵。
5. 通过贪心排序选择目标，并交给 Nav2 执行。

该实现采用贪心求解，不保证 MRTSP 全局最优。模块还提供前沿、选中目标和优化地图的调试话题，以及地图处理、前沿检测、代价计算和目标切换等测试文件。

## 环境与依赖

- **ROS 2**：多数子项目以 Humble 为基础；MRTSP 上游标注 Jazzy，本仓库快速上手文档记录了 Humble 适配过程。各模块兼容范围需分别核对。
- **构建工具**：`colcon`、`rosdep`、C++ 编译环境和 Python 3。
- **运行环境**：Nav2、SLAM（例如 `slam_toolbox`）、有效的 TF 树，以及实体机器人或仿真环境。
- **可选仿真与显示**：TurtleBot3、Gazebo、RViz。
- **Python 依赖**：MRTSP 的依赖声明包含 NumPy、SciPy 和 OpenCV，安装前应核对当前 Python 环境的版本兼容性。

探索节点通常需要以下接口；具体名称可由参数或重映射调整：

| 接口 | 用途 |
| --- | --- |
| `/map` | SLAM 输出的占用栅格地图 |
| `/global_costmap/costmap` | 全局导航代价地图，部分算法使用 |
| `/local_costmap/costmap` | 局部导航代价地图，部分算法使用 |
| `map → base_link` 或 `map → base_footprint` | 地图到机器人基座的 TF 变换链 |
| `/navigate_to_pose` | Nav2 导航动作服务器 |
| `/clock` | 仿真时间，启用 `use_sim_time` 时使用 |

## 构建

以下以 ROS 2 Humble 为例，在工作区根目录执行。需要先安装 ROS 2、`rosdep` 和 `colcon`，并完成 `rosdep` 初始化。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

也可以仅构建所需模块及其工作区依赖，例如：

```bash
colcon build --symlink-install --packages-up-to explore_lite
```

## 启动探索

先启动机器人或仿真、SLAM 和 Nav2，并确认地图与 TF 正常。每个新终端都需要加载 ROS 2 环境和本工作区的 `install/setup.bash`。

以下探索节点是不同策略的启动入口。对同一台机器人，每次选择一种策略运行，避免多个节点同时向 Nav2 发送目标。仿真使用 `use_sim_time:=true`，实体机器人通常使用 `false`。

**占用栅格前沿探索：**

```bash
ros2 run frontier_exploration frontier_planner --ros-args -p use_sim_time:=true
```

**RRT 探索：**

```bash
ros2 launch rrt_explore rrt.launch.py namespace:=robot1
ros2 param set /robot1/rrt use_sim_time true
ros2 topic pub --once /robot1/rrt/enable std_msgs/msg/Bool '{data: true}'
```

RRT 的当前 launch 未提供 `use_sim_time` 参数，因此这里在启用探索前设置节点参数。其单机器人 Nav2 接入还需要允许规划器穿越未知区域，参见子项目快速上手文档及 `config/nav2_params_explore.yaml`。

**轻量前沿探索：**

```bash
ros2 launch explore_lite explore.launch.py use_sim_time:=true
```

**MRTSP 探索：**

```bash
ros2 launch mrtsp_exploration_ros2 explore.launch.py use_sim_time:=true
```

MRTSP 默认参数位于 `src/mrtsp_exploration_ros2/config/params.yaml`，也可以通过 `params_file:=/absolute/path/to/params.yaml` 指定参数文件。

**HGVD 拓扑探索：**

```bash
ros2 launch hgvd_exploration explore.launch.py use_sim_time:=true rviz:=true
```

Gazebo 与 SLAM、Nav2 联调时，使用上面的探索节点入口。下文的 `benchmark.launch.py` 会启动自己的矩阵世界和导航动作服务器。

HGVD 使用分层广义 Voronoi 图、带连通性约束的聚类和拓扑距离选择可达前沿视点。加上 `return_to_start:=true` 可启用返航，加上 `navigation_enabled:=false` 可静止预览拓扑。构建、接口、计时和测试说明见 [HGVD README](src/hgvd-exploration/README.md)。

**多机器人地图合并：**

```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

地图合并需要预先启动各机器人的建图节点，并配置对应的地图命名空间和初始位姿选项。

## 无 Gazebo 的二维矩阵测试

新增的 `src/explore_grid_benchmark/` 通过独立 ROS 接口测试现有探索节点，无需修改探索算法源码，也不需要启动 Gazebo、SLAM 或真实 Nav2。支持 YAML、CSV、NumPy 二维矩阵，提供受墙体遮挡的观测、机器人运动、导航目标执行和指标保存。

```bash
# 构建测试器与五种探索算法
colcon build --symlink-install --packages-up-to explore_grid_benchmark \
  frontier_exploration rrt_explore explore_lite mrtsp_exploration_ros2 hgvd_exploration
source install/setup.bash

# 单独测试一种算法，可选 frontier / rrt / explore_lite / mrtsp / hgvd
ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=hgvd rviz:=true

# 依次测试默认的四种算法，每轮重置环境
ros2 launch explore_grid_benchmark compare.launch.py

# 按场景 × 算法批量测试
ros2 launch explore_grid_benchmark suite.launch.py \
  scenarios:=open,rooms,corridor,clutter algorithms:=rrt,mrtsp,hgvd
```

内置 `open`、`rooms`、`corridor`、`clutter` 四个场景，通过 `scenario:=corridor` 选择，或传入自定义场景 YAML 路径。也可通过 `map_file:=/absolute/path/to/map.yaml` 直接指定矩阵地图。矩阵值为 `0`（空闲）与 `100`（墙体），最外圈为墙；未知区域由传感器模型逐步揭示。实验默认使用独立的 ROS domain 71，结果保存到 `results/grid_benchmark/`，包含覆盖率、路径长度、目标状态和轨迹。

模型采用点机器人、四连通 A* 与全向距离观测，测量结果适用于该简化环境。详细输入格式、参数和测试方法见[二维矩阵测试器说明](src/explore_grid_benchmark/README.md)。

## 文档与当前注意事项

- [占用栅格前沿探索说明](src/frontier-occ-exploration/README.md)
- [RRT 探索说明](src/rrt-explore-ros2/README.md)与[快速上手](src/rrt-explore-ros2/docs/quickstart.md)
- [explore_lite 与多机器人地图合并说明](src/m-explore-ros2/README.md)
- [MRTSP 算法与接口说明](src/mrtsp_exploration_ros2/README.md)与[Humble 快速上手](src/mrtsp_exploration_ros2/docs/quickstart.md)
- [ROS 2 性能采样与火焰图指南](docs/perf_ros2_profiling.md)

当前 `frontier_sim.launch.py` 中，Gazebo、机器人和 Nav2 的启动部分已被注释，主要启用 RViz 与探索节点，使用前需要单独准备仿真与导航环境。

部分子项目文档保留了原工作区的路径示例，使用时应替换为本地实际路径。本 README 根据仓库源码与配置整理，不代表所有模块已在同一 ROS 2 环境下完成联合运行验证。

各子项目保留各自的许可证，使用与分发时请查阅对应目录中的许可证文件及源码声明。
