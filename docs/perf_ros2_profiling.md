# 使用 perf 测量 ROS 2 进程与生成火焰图

本文介绍如何测量 ROS 2 节点的 CPU 性能，并生成函数级火焰图。

示例以本工作区的旧版探索节点为例：

```text
frontier_exploration/frontier_planner
```

同样的方法也适用于其他 ROS 2 节点。

## 1. 前置条件

安装 `perf`：

```bash
sudo apt update
sudo apt install linux-tools-common linux-tools-$(uname -r)
```

确认安装成功：

```bash
perf --version
```

建议使用带调试信息的构建版本，以便火焰图显示函数名：

```bash
cd ~/ros_ws/frontier_exploration_ws

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select frontier_exploration \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer

source install/setup.bash
```

`-fno-omit-frame-pointer` 有助于 `perf` 正确展开调用栈。

## 2. 启动 ROS 2 仿真

建议分开启动各个系统组件，并将日志写入可写目录：

终端 1：Gazebo

```bash
export TURTLEBOT3_MODEL=burger
export ROS_LOG_DIR=$PWD/perf_results/ros_log
mkdir -p perf_results/ros_log

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

终端 2：Nav2

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=$PWD/perf_results/ros_log

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

终端 3：SLAM Toolbox

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=$PWD/perf_results/ros_log

ros2 launch slam_toolbox online_async_launch.py
```

终端 4：探索节点

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch frontier_exploration frontier_sim.launch.py
```

确认节点和主要话题存在：

```bash
ros2 node list
ros2 topic list | grep -E '^/(map|scan|odom|tf|global_costmap|local_costmap)'
```

必须确认至少存在：

```text
/gazebo
/slam_toolbox
/frontier_planner
/map
/scan
/tf
/global_costmap/costmap
```

## 3. 获取需要测量的进程 PID

不要测量 `ros2 launch` 外层 Python 进程，应测量真正的节点进程：

```bash
pgrep -af frontier_planner
```

或者：

```bash
ps -eo pid,ppid,pgid,etime,cmd | grep frontier_planner
```

设置 PID 环境变量：

```bash
PID=$(pgrep -n -f '/frontier_planner')
echo "$PID"
```

确认进程确实属于目标节点：

```bash
ps -p "$PID" -o pid,ppid,pgid,etime,cmd
```

## 4. 使用 perf stat 统计进程性能

### 4.1 基本统计

对目标节点采样 120 秒：

```bash
mkdir -p perf_results

perf stat \
  -p "$PID" \
  -d \
  -o perf_results/frontier_planner.stat.txt \
  -- sleep 120
```

### 4.2 重复测量

使用 `-r 3` 重复三次：

```bash
perf stat \
  -p "$PID" \
  -d \
  -r 3 \
  -o perf_results/frontier_planner.stat.repeat.txt \
  -- sleep 120
```

常用指标包括：

| 指标 | 含义 |
|---|---|
| `task-clock` | 进程占用的 CPU 时间 |
| `context-switches` | 上下文切换次数 |
| `cycles` | CPU 周期数 |
| `instructions` | 执行的指令数 |
| `branches` | 分支指令数 |
| `branch-misses` | 分支预测失败次数 |
| cache-misses | CPU 缓存未命中次数 |

也可以指定事件：

```bash
perf stat \
  -p "$PID" \
  -e task-clock,context-switches,cpu-cycles,instructions,branches,branch-misses \
  -- sleep 120
```

## 5. 使用 perf record 采集火焰图数据

以 99 Hz 采样，并记录用户态调用栈：

```bash
perf record \
  -F 99 \
  -g \
  --call-graph dwarf \
  -p "$PID" \
  -o perf_results/frontier_planner.perf.data \
  -- sleep 120
```

参数说明：

| 参数 | 说明 |
|---|---|
| `-F 99` | 每秒采样 99 次 |
| `-g` | 记录调用栈 |
| `--call-graph dwarf` | 使用 DWARF 展开调用栈 |
| `-p PID` | 附加到指定进程 |
| `-o` | 指定输出文件 |

查看交互式热点报告：

```bash
perf report -i perf_results/frontier_planner.perf.data
```

导出文本调用栈：

```bash
perf script \
  -i perf_results/frontier_planner.perf.data \
  > perf_results/frontier_planner.perf.script
```

## 6. 生成火焰图

下载 Brendan Gregg 的 FlameGraph 工具：

```bash
git clone https://github.com/brendangregg/FlameGraph.git tools/FlameGraph
```

生成折叠调用栈：

```bash
tools/FlameGraph/stackcollapse-perf.pl \
  perf_results/frontier_planner.perf.script \
  > perf_results/frontier_planner.folded
```

生成 SVG 火焰图：

```bash
tools/FlameGraph/flamegraph.pl \
  --title='frontier_planner CPU flame graph' \
  --countname='samples' \
  perf_results/frontier_planner.folded \
  > perf_results/frontier_planner.flame.svg
```

在浏览器中打开：

```bash
xdg-open perf_results/frontier_planner.flame.svg
```

火焰图中：

- 横向宽度表示 CPU 采样占比；
- 越宽的函数消耗 CPU 越多；
- 从下到上表示调用关系；
- 可以重点检查 `InflateMap`、`ComputeFrontier`、`ComputeCentroids`、`SelectGoal` 和 `MoveToGoal`。

## 7. 在 launch 文件中直接使用 perf

也可以在 ROS 2 launch 的 `Node` 中设置 `prefix`：

```python
Node(
    package='frontier_exploration',
    executable='frontier_planner',
    name='frontier_planner',
    prefix=(
        'perf record -F 99 -g --call-graph dwarf '
        '-o /tmp/frontier_planner.perf.data --'
    ),
    output='screen',
)
```

仿真结束后查看：

```bash
perf report -i /tmp/frontier_planner.perf.data
```

注意：`prefix` 中的 `perf` 只会测量这个 `Node` 进程。Gazebo、Nav2 和 SLAM 应分别测量，或者使用系统级采样。

## 8. 同时测量多个 ROS 2 进程

分别获取 PID：

```bash
GAZEBO_PID=$(pgrep -n -f 'gzserver')
SLAM_PID=$(pgrep -n -f 'async_slam_toolbox_node')
FRONTIER_PID=$(pgrep -n -f 'frontier_planner')
```

分别采样：

```bash
perf record -F 99 -g --call-graph dwarf \
  -p "$GAZEBO_PID" \
  -o perf_results/gazebo.perf.data \
  -- sleep 120 &

perf record -F 99 -g --call-graph dwarf \
  -p "$SLAM_PID" \
  -o perf_results/slam_toolbox.perf.data \
  -- sleep 120 &

perf record -F 99 -g --call-graph dwarf \
  -p "$FRONTIER_PID" \
  -o perf_results/frontier_planner.perf.data \
  -- sleep 120 &

wait
```

如果目标是分析整个系统，也可以使用系统级采样：

```bash
sudo perf record \
  -F 99 \
  -a \
  -g \
  --call-graph dwarf \
  -o perf_results/system.perf.data \
  -- sleep 120
```

系统级采样会包含桌面、RViz、Gazebo 和其他进程，生成火焰图前应使用进程名或函数名筛选。

## 9. 常见问题

### 9.1 `Permission denied`

查看当前限制：

```bash
cat /proc/sys/kernel/perf_event_paranoid
```

临时降低限制：

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

如果仍然失败：

```bash
sudo sysctl kernel.perf_event_paranoid=-1
```

### 9.2 火焰图中只有地址，没有函数名

通常是因为没有调试符号或调用栈无法展开。重新编译：

```bash
colcon build \
  --packages-select frontier_exploration \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer
```

然后重新运行 `perf record`。

### 9.3 采样对象是 `ros2 launch` 而不是节点

不要使用：

```bash
pgrep -n ros2
```

应使用：

```bash
pgrep -af frontier_planner
```

`ros2 launch` 只是启动器，真正的 CPU 消耗通常发生在 `frontier_planner`、`async_slam_toolbox_node` 或 Nav2 节点中。

## 10. 结束仿真并清理节点

优先结束对应的 launch 进程组。如果使用了独立终端，按 `Ctrl-C` 结束每个 launch。

检查是否仍有相关进程：

```bash
ps -eo pid,ppid,pgid,stat,cmd \
  | grep -E 'gzserver|gzclient|frontier_planner|slam_toolbox|nav2|rviz2' \
  | grep -v grep
```

确认节点是否清空：

```bash
ros2 node list
```

不要随意杀掉不属于本次仿真的 ROS 进程；应先通过 PID、PGID 和命令行确认目标。

