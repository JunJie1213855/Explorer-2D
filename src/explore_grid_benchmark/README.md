# 二维矩阵探索测试器

独立于现有算法的 ROS 2 测试包，无需 Gazebo、SLAM 或真实 Nav2。提供逐步揭示的占用栅格地图、TF、模拟 `NavigateToPose` 动作服务和 `/cmd_vel` 运动处理，运行现有探索节点。

## 构建与启动

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to explore_grid_benchmark \
  frontier_exploration rrt_explore explore_lite mrtsp_exploration_ros2
source install/setup.bash
ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=mrtsp rviz:=true
```

`algorithm` 支持 `frontier`、`rrt`、`explore_lite`、`mrtsp`；`none` 仅启动模拟器，用于检查地图和接口。每次只启动一个算法。无需手动启用 RRT。

也可以用一个 launch 顺序测试全部四种算法，每轮重置地图与起点：

```bash
ros2 launch explore_grid_benchmark compare.launch.py max_duration:=180.0 rviz:=true
```

`compare.launch.py` 接收下表中的公共参数（不含 `algorithm`），每轮使用相同设置，总运行时间包含四轮实验以及各轮启动和退出清理时间。节点未及时退出时，launch 会按标准流程升级退出信号。

默认使用 ROS domain **71**，与通常的 domain 0 分开。并行实验必须指定不同 `domain_id`，确保选择的 domain 没有其他机器人或实验进程。

```bash
ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=explore_lite \
  map_file:=/absolute/path/to/room.yaml max_duration:=180.0
# 在另一个终端查看指标
ROS_DOMAIN_ID=71 ros2 topic echo /benchmark/metrics
```

## 多场景配置

场景 YAML 放在 `scenarios/`，将地图路径、起点覆盖值、感知范围、速度和结束条件组织在一起。

| 场景名 | 内容 | 默认最长时间 |
| --- | --- | --- |
| `open` | 无内部障碍的空旷房间 | 180 秒 |
| `rooms` | 带门洞的房间隔断和障碍物，默认场景 | 180 秒 |
| `corridor` | 通过交错门洞连接的三段走廊区域 | 240 秒 |
| `clutter` | 四组障碍物形成的遮挡区域 | 180 秒 |

```bash
# 一个场景、一个算法
ros2 launch explore_grid_benchmark benchmark.launch.py scenario:=open algorithm:=mrtsp rviz:=true

# 一个场景、全部四种算法
ros2 launch explore_grid_benchmark compare.launch.py scenario:=corridor

# 多个场景与多个算法依次组合；以下共运行 4 轮
ros2 launch explore_grid_benchmark suite.launch.py \
  scenarios:=open,corridor algorithms:=rrt,mrtsp max_duration:=60.0
```

不传列表时，`suite.launch.py` 默认运行四个场景与四种算法的全部 16 种组合。每轮重置环境，先运行同一场景的各算法，再切换下一场景；总耗时包含所有轮次及进程清理时间。需要短时检查时可显式降低 `max_duration`。

自定义场景示例（与原有地图 YAML 分开）：

```yaml
name: my_room
description: 自定义房间配置
map_file: ../maps/my_room.yaml
simulation:
  start_x: 5
  start_y: 5
  sensor_range: 1.2
  speed: 0.6
  angular_speed: 1.5
  max_duration: 240.0
  stall_timeout: 60.0
  coverage_target: 0.98
```

`map_file` 相对场景 YAML 所在目录解析，起点仍为 `[列, 行]`。可通过 `scenario:=/absolute/path/to/my_scene.yaml` 使用外部场景，无需重新构建。批量测试的 `scenarios` 列表也支持逗号分隔的外部场景 YAML 路径，路径本身不能含逗号。

参数优先级为 **显式 launch 参数 > 场景 simulation 配置 > 测试器默认值**。例如 `scenario:=corridor sensor_range:=2.0` 覆盖走廊场景的感知范围；显式 `map_file` 会覆盖场景地图，在 suite 中对所有轮次生效。`resolution`、`start_x`、`start_y` 未指定时继续使用地图元数据。

每轮结果目录格式为 `场景名_算法名_时间戳_随机后缀`，`summary.json` 包含场景名、场景文件路径及实际生效参数。内置预设修改后需要重新构建；个人配置可保存在外部 YAML。

## 输入矩阵

支持 YAML、CSV、NumPy `.npy`，其中矩阵值为 `0`（空闲）或 `100`（墙体）。真实地图不含 `-1`；未知区域由模拟观测生成。最外圈必须为墙体，至少 3×3。机器人起点必须为空闲格。

YAML 示例：

```yaml
resolution: 0.2          # 米/格
origin: [0.0, 0.0]       # 地图左下角世界坐标
start: [2, 2]           # [列 x, 行 y]，从 0 开始
start_yaw: 0.0          # 弧度
# 第 0 行在世界坐标最下方，各行沿 +y 递增。
grid:
  - [100, 100, 100, 100, 100, 100]
  - [100,   0,   0,   0,   0, 100]
  - [100,   0,   0,   0,   0, 100]
  - [100,   0,   0,   0,   0, 100]
  - [100, 100, 100, 100, 100, 100]
```

常规图像数组的第 0 行在上方，转换前可使用 `np.flipud(image)`，并将像素转换成 `0/100`。`.npy` 禁止 pickle 对象；CSV 不带表头。

```bash
ros2 launch explore_grid_benchmark benchmark.launch.py algorithm:=rrt \
  map_file:=/absolute/path/to/room.npy resolution:=0.2 start_x:=5 start_y:=5
```

CSV/NPY 无元数据时默认分辨率为 0.2、起点为 `[1, 1]`。launch 中 `resolution:=0.0` 使用文件元数据，`start_x`、`start_y` 默认 `-1`，成对指定时覆盖文件起点。

## 模型与参数

- 机器人是点模型，采用四连通 A*，只根据已观测地图规划，允许经过未知格；遇到新障碍重新规划。目标落在已知障碍或无路可达时返回失败。
- 360° 有限距离观测，射线被墙遮挡；不模拟激光噪声、SLAM 误差或动力学。旋转会更新 TF，但不会增加全向传感器的可见范围。
- 运动先转向再平移，按墙钟时间执行。所有节点 `use_sim_time=false`，不需要 `/clock`。
- 目标按所在格中心执行；导航替换旧目标时，旧目标记为 `preempted` 并返回 aborted，新目标接管运动。支持显式取消；这是简化动作服务，与真实 Nav2 的全部语义不完全相同。
- `inflation_radius` 在已观测墙周围生成二值阻挡区域，默认 0；不是实际机器人足迹。`explore_lite` 默认对 `/map` 搜索前沿，其他算法也有自己的前沿处理，参数并非数学等价。

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `algorithm` | `mrtsp` | 运行的探索算法 |
| `scenario` | `rooms` | 内置场景名或外部场景 YAML 路径 |
| `map_file` | 内置 `rooms.yaml` | 矩阵文件绝对路径 |
| `sensor_range` | `1.5` | 观测半径，米 |
| `speed` | `0.8` | 平移速度，米/秒 |
| `angular_speed` | `1.5` | 角速度上限，弧度/秒 |
| `inflation_radius` | `0.0` | 导航阻挡膨胀半径，米；frontier 节点也使用该膨胀参数 |
| `max_duration` | `180.0` | 最长运行时间，秒 |
| `stall_timeout` | `45.0` | 已观测可达空闲格连续不增长的最长时间，秒 |
| `coverage_target` | `0.98` | 覆盖率达到该值即结束 |
| `results_dir` | 当前目录下 `results/grid_benchmark` | 结果根目录 |
| `rviz` | `false` | 打开 RViz |
| `domain_id` | `71` | 测试专用 ROS domain |

参数表中的数值描述默认 `rooms` 场景的生效值；选择其他场景时以场景配置为准。

覆盖率 = 已观测的起点可达空闲格 / 真实地图中起点四连通可达空闲格。真实地图只用于观测、碰撞检查、覆盖率计算及调试显示，不用于规划和前沿提取。

停止原因是 `coverage_target`、`timeout`、`stalled` 或用户中断 `interrupted`。覆盖率停止条件独立于算法自身宣布完成的条件，`stalled` 也不代表算法已完成。模拟器结束后 launch 关闭本次启动的其他节点。

原 MRTSP 节点在 Humble 下收到退出信号时可能报告 `rcl_shutdown already called`；这是该节点的退出清理问题，测试结果在关闭算法进程前已经保存。本包保留其源码。

## 输出与对比

每次实验生成带算法名、时间戳和随机后缀的目录：

- `summary.json`：实验参数、结束原因、覆盖率、路径长度、碰撞尝试和目标执行计数。
- `metrics.csv`：约每秒采样一次的指标。
- `trajectory.csv`：约 5 Hz 的机器人位置与朝向。
- `goals.csv`：接收的导航目标及最终状态；非法请求会被拒绝，不计入此表。
- `observed.npy`：结束时的已观测地图（含 `-1` 未知格）。

`/map` 是算法输入，`/global_costmap/costmap` 与 `/local_costmap/costmap` 是观测代价地图；`/benchmark/truth` 仅用于人工调试，不应重映射到算法输入。RViz 默认显示观测地图、运动轨迹、机器人坐标系和各算法前沿。

对比时保持同一地图、起点、感知、速度和结束条件，逐一运行四种算法，比较覆盖率曲线、路径长度、失败次数与结束原因。耗时是本简化环境中的墙钟时间，包含转向、运动及计算等待；不等同真实机器人性能。RRT 的随机初始化由原算法决定，本测试器不提供可控随机种子，建议重复实验。

## 验证

纯栅格测试无需 ROS：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/explore_grid_benchmark \
  python3 -m pytest -q src/explore_grid_benchmark/test
```

加载 ROS 环境后，可在独立 domain 运行额外的动作接口测试，验证成功、取消、目标替换、障碍目标失败和越界拒绝：

```bash
ROS_DOMAIN_ID=75 EXPLORER_ROS_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=src/explore_grid_benchmark:$PYTHONPATH \
  python3 -m pytest -q src/explore_grid_benchmark/test
```

不修改各探索包源码；本包只通过 ROS 接口、节点参数和重映射接入现有算法。

验证包含栅格模型、场景读取与覆盖优先级、批量任务展开及 ROS 动作接口。四种算法均进行了短时接口与运动接入检查；短时检查用于验证接入，不作为算法性能排名，RViz 配置尚未进行桌面显示验证。
