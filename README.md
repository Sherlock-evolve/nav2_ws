# nav2_lab 项目总览

## 1. 项目定位

`nav2_lab` 是一个以仿真为起点的 Nav2 学习与实验项目，基于以下组件搭建：

- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Waffle
- Nav2
- AMCL
- SLAM Toolbox

这是当前工作区中的主项目，用来做可重复启动、自动任务执行、参数调优和后续功能扩展。

当前这一版可以视为 `v1` 基线，目标很明确：

> 拉起仿真，加载地图，完成定位，自动下发导航目标，并记录运行结果。

## 2. 项目结构

当前工作区里和这个项目直接相关的主要包有三个。

### 2.1 `nav2_lab_bringup`

这是顶层 bringup 包，负责整体启动和基础资源管理。

职责：

- 启动 Gazebo 仿真
- 启动 Nav2 定位与导航栈
- 提供默认参数文件
- 提供默认地图
- 提供 RViz 配置
- 提供实验总入口

主要目录和文件：

- `launch/sim.launch.py`
- `launch/slam.launch.py`
- `launch/navigation.launch.py`
- `launch/lab.launch.py`
- `params/nav2_params.yaml`
- `maps/simple_room.yaml`
- `maps/simple_room.pgm`
- `rviz/`

### 2.2 `nav2_lab_worlds`

这个包存放 Gazebo Classic 世界文件。

当前已有世界：

- `worlds/simple_room.world`
- `worlds/narrow_corridor.world`

用途：

- `simple_room.world`：当前基线验证场景
- `narrow_corridor.world`：后续做代价地图、局部规划器、参数调优实验

### 2.3 `nav2_lab_missions`

这个包负责自动任务执行与运行过程记录。

职责：

- 发布 `/initialpose`
- 调用 Nav2 `NavigateToPose` action
- 等待定位完成和导航栈激活
- 顺序执行多个导航目标
- 输出任务结果 CSV
- 输出遥测日志 CSV

主要文件：

- `config/simple_room_mission.yaml`
- `nav2_lab_missions/mission_runner.py`
- `nav2_lab_missions/mission_logger.py`

## 3. 核心源码说明

### 3.1 `lab.launch.py`

路径：

- `nav2_lab_bringup/launch/lab.launch.py`

这是日常使用最核心的总入口。

它主要做三件事：

1. 启动 Gazebo 与机器人模型
2. 启动 Nav2 定位和导航栈
3. 按需启动 `mission_runner` 和 `mission_logger`

关键参数：

- `world`
- `map`
- `params_file`
- `mission_file`
- `run_mission`
- `use_sim_time`
- `use_rviz`

### 3.2 `navigation.launch.py`

路径：

- `nav2_lab_bringup/launch/navigation.launch.py`

这个启动文件封装了 `nav2_bringup/bringup_launch.py`，负责真正把 Nav2 启起来。

职责：

- 加载静态地图
- 启动 AMCL
- 启动 planner / controller / behavior / BT 等导航节点
- 启动 RViz

当前默认 RViz 配置已经切换为官方配置：

- `/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz`

这样做的原因是之前那个极简 RViz 配置不够完整，手动调试时地图、目标点交互都不稳定。

### 3.3 `mission_runner.py`

路径：

- `nav2_lab_missions/nav2_lab_missions/mission_runner.py`

这是自动任务节点的核心实现。

当前逻辑：

1. 读取 mission YAML
2. 等待 `navigate_to_pose` action server
3. 等待 `/initialpose` 的订阅端就绪
4. 在短时间窗口内重复发布初始位姿
5. 等待 `/amcl_pose`
6. 再额外等待几秒，确保 Nav2 完全激活
7. 依次发送导航目标
8. 把状态、耗时、反馈写入 CSV

这个节点不会直接发布 `/cmd_vel` 控制机器人。

它只负责：

- 发布 `/initialpose`
- 监听 `/amcl_pose`
- 通过 `NavigateToPose` action 发目标

真正的运动控制仍然由 Nav2 内部完成。

### 3.4 `mission_logger.py`

路径：

- `nav2_lab_missions/nav2_lab_missions/mission_logger.py`

这是一个轻量级遥测记录节点。

它订阅：

- `/cmd_vel`
- `/amcl_pose`
- `/navigate_to_pose/_action/status`

输出文件位置：

- `/tmp/nav2_lab_results/*_telemetry.csv`

## 4. 任务配置

当前任务配置文件：

- `nav2_lab_missions/config/simple_room_mission.yaml`

当前字段包括：

- `frame_id`
- `default_timeout_sec`
- `retry_count`
- `initial_pose`
- `goals`

当前默认任务点包括：

1. `east_side`
2. `north_west`
3. `home`

配置示例：

```yaml
frame_id: map
default_timeout_sec: 90.0
retry_count: 1
initial_pose:
  x: -1.2
  y: -1.2
  yaw: 0.0
goals:
  - name: east_side
    x: 1.25
    y: -1.05
    yaw: 0.0
```

## 5. 运行数据流

当前自动任务模式下，核心数据流如下：

```text
Gazebo TurtleBot3
  -> 发布 /scan, /odom, /tf
  -> map_server 发布 /map
  -> mission_runner 发布 /initialpose
  -> AMCL 消费 /map + /scan + /odom + /tf
  -> AMCL 发布 /amcl_pose 和 map -> odom
  -> mission_runner 发送 NavigateToPose 目标
  -> bt_navigator 组织整套导航行为
  -> planner_server 生成全局路径
  -> controller_server 计算控制量
  -> /cmd_vel
  -> Gazebo 差速底盘执行运动
```

这里最关键的边界是：

- `mission_runner` 是任务层目标发送器
- Nav2 是导航决策与控制栈
- 机器人最终运动依然来自 Nav2 输出的 `/cmd_vel`

所以你之前看到机器人动起来，不是我在外部直接发速度话题，而是 `mission_runner` 通过 Nav2 action 发目标，Nav2 自己规划并输出控制命令。

## 6. 启动方法

### 6.1 编译

```bash
cd ~/Desktop/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如需统一日志目录，也可以加上：

```bash
export ROS_LOG_DIR=/tmp/ros_logs
```

### 6.2 只启动仿真

```bash
ros2 launch nav2_lab_bringup sim.launch.py
```

常用参数示例：

```bash
ros2 launch nav2_lab_bringup sim.launch.py world:=narrow_corridor
ros2 launch nav2_lab_bringup sim.launch.py x_pose:=-1.2 y_pose:=-1.2 yaw:=0.0
```

### 6.3 启动导航栈

保持仿真运行，在另一个终端执行：

```bash
ros2 launch nav2_lab_bringup navigation.launch.py
```

### 6.4 启动完整实验入口

```bash
ros2 launch nav2_lab_bringup lab.launch.py run_mission:=true
```

调试时更推荐显式关闭组件容器模式：

```bash
ros2 launch nav2_lab_bringup lab.launch.py run_mission:=true use_composition:=False
```

这样日志更直观，排查问题更方便。

### 6.5 手动 RViz 测试

```bash
ros2 launch nav2_lab_bringup lab.launch.py run_mission:=false
```

手动测试流程：

1. 用 `2D Pose Estimate` 设置初始位姿
2. 等 AMCL 稳定
3. 用 `Nav2 Goal` 发送目标点

如果 `run_mission:=false`，系统不会自动下发任务点。

## 7. 当前验证结果

这一版已经完成过一次完整自动任务验证。

成功结果示例：

```csv
goal_name,attempt,status,duration_sec,message
east_side,1,succeeded,16.861,distance_remaining=0.071
north_west,1,succeeded,39.231,distance_remaining=0.000
home,1,succeeded,30.261,distance_remaining=0.000
```

结果文件示例：

- `/tmp/nav2_lab_results/20260630_172336_mission.csv`

遥测文件示例：

- `/tmp/nav2_lab_results/20260630_172335_telemetry.csv`

这说明当前基线已经具备：

- 启动仿真
- 装载地图
- 完成定位
- 自动发送目标
- 连续完成三段导航
- 记录实验结果

## 8. 已经发现并修过的问题

### 8.1 任务发送过早

问题：

- `mission_runner` 早于 Nav2 完全激活就发送目标
- `NavigateToPose` 会直接 reject

修复：

- 等待 `/amcl_pose`
- 再额外等待一段激活稳定时间

### 8.2 初始位姿竞争条件

问题：

- `/initialpose` 可能发得太早
- AMCL 还没完成订阅发现，导致初始位姿丢失

修复：

- 先等待 `/initialpose` 订阅端就绪
- 在短时间内重复发布初始位姿

### 8.3 RViz 过于简化

问题：

- 自定义 RViz 配置太薄
- 手动调试时地图和目标交互不稳定

修复：

- 默认切到 Nav2 官方 `nav2_default_view.rviz`

## 9. 当前限制

虽然这一版已经能跑通，但它还是一个刻意保持简单的基线版本。

当前限制包括：

- 目前只有一个主基线地图
- `simple_room.yaml` 还是手工准备的基线地图，还不是严格由当前世界重新 SLAM 生成
- 任务统计还比较基础
- 还没有参数实验矩阵
- 还没有自定义 planner / controller / BT 插件
- 还没有动态障碍物实验

## 10. 后续开发方向

如果你要继续进阶，比较合理的推进顺序是下面这条线。

### 10.1 先稳住基线

- 固化一套标准启动流程
- 把 `simple_room` 作为标准参考场景
- 固化预期 RViz 现象和 CSV 输出样例

### 10.2 加实验变体

- 生成与 `narrow_corridor.world` 对应的地图
- 增加 corridor 专用 mission YAML
- 对比不同参数下的成功率、路径质量、耗时

### 10.3 加参数调优资产

先建立多套参数版本，重点观察：

- `inflation_radius`
- `controller_frequency`
- `max_vel_x`
- `xy_goal_tolerance`
- local/global costmap 尺寸

### 10.4 加诊断与统计

- 统计 recovery 触发次数
- 汇总任务总耗时
- 汇总重试次数
- 更明确记录失败原因

### 10.5 再进入真正的扩展开发

当这条基线足够稳定后，最值得做的工程化进阶方向是：

- 自定义 BT XML
- 自定义 recovery behavior
- 自定义 costmap layer
- 简单自定义 planner plugin

## 11. 实用总结

现在的 `nav2_lab` 已经不只是一个“能启动 Nav2 的包”。

它已经是一个可工作的微型导航实验平台，具备：

- 明确拆分的包结构
- 可重复的启动入口
- 自动任务执行
- 结果记录
- 已验证的三目标自主导航基线

它的价值不在“功能很多”，而在“结构清楚、链路跑通、便于继续扩展”。

如果下一阶段你要继续往上走，最合适的路线不是继续堆 launch，而是围绕下面三件事展开：

1. 做标准化实验场景
2. 做参数对比与结果统计
3. 做一个真正属于你自己的 Nav2 扩展模块
