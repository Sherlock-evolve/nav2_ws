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
- `launch/explore.launch.py`
- `launch/lab.launch.py`
- `params/nav2_params.yaml`
- `maps/simple_room.yaml`、`maps/simple_room.pgm`（基线室内场景）
- `maps/real.yaml`、`maps/real.pgm`（`real.world` 的 SLAM 建图成果）
- `rviz/`

### 2.2 `nav2_lab_worlds`

这个包存放 Gazebo Classic 世界文件。

当前已有世界：

- `worlds/simple_room.world`
- `worlds/real.world`

用途：

- `simple_room.world`：当前基线验证场景，小型室内房间
- `real.world`：更大的拟真实环境场景，对应 `maps/real.*` 地图和 `real_world_mission.yaml` 任务

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

- `world`：世界名（解析为 `nav2_lab_worlds/worlds/<world>.world`）或绝对 `.world` 路径
- `model`：TurtleBot3 模型，默认 `waffle`
- `x_pose` / `y_pose` / `yaw`：机器人初始位姿，默认 `(-1.2, -1.2, 0.0)`
- `map`：地图名（解析为 `nav2_lab_bringup/maps/<map>.yaml`）或绝对 `.yaml` 路径，默认 `simple_room`
- `params_file`：Nav2 参数文件
- `mission`：任务名（解析为 `nav2_lab_missions/config/<mission>.yaml`），默认 `simple_room_mission`
- `mission_file`：可选，任务名或路径，设置时覆盖 `mission`
- `slam`：是否以 SLAM 模式启动 Nav2（默认 `False`，走 AMCL）
- `run_mission`
- `use_sim_time`
- `use_rviz`

实现说明：`lab.launch.py` 现在用 `OpaqueFunction` 包裹，先声明参数再在运行期把 `map` / `mission` 这类「名字」解析成真实路径（`_resolve_config_file`）。解析顺序是：绝对路径或 `~` 展开 → install 目录里的文件 → symlink-install 情况下回退到源码目录。`navigation.launch.py` 的 `_resolve_map` 也是同样的逻辑。

### 3.2 `navigation.launch.py`

路径：

- `nav2_lab_bringup/launch/navigation.launch.py`

这个启动文件封装了 `nav2_bringup/bringup_launch.py`，负责真正把 Nav2 启起来。

职责：

- 加载静态地图
- 启动 AMCL
- 启动 planner / controller / behavior / BT 等导航节点
- 启动 RViz

它暴露 `slam` 参数：默认 `False` 走 AMCL 定位，设为 `True` 时把 Nav2 切到在线 SLAM（slam_toolbox）。`lab.launch.py` 会把同名的 `slam` 参数透传进来。

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
3. 在短时间窗口内重复发布初始位姿（可被 `publish_initial_pose: false` 跳过）
4. 等待 `/initialpose` 的订阅端就绪
5. 等待 `/amcl_pose`（可被 `wait_for_localization: false` 跳过，此时只等激活延时）
6. 再额外等待几秒，确保 Nav2 完全激活
7. 依次发送导航目标
8. 把状态、耗时、反馈写入 CSV

新增的两个可选 mission 字段（默认都是 `true`，对 `simple_room_mission.yaml` 行为不变）：

- `publish_initial_pose`：是否由 `mission_runner` 发布 `/initialpose`，关闭后适合已有外部初始位姿或纯 SLAM 模式
- `wait_for_localization`：是否等待 `/amcl_pose`，关闭后只按 `nav_activation_delay_sec` 等待 Nav2 激活

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

### 3.5 `explore_runner.py`

路径：

- `nav2_lab_missions/nav2_lab_missions/explore_runner.py`

这是自动探索建图节点，用来替掉 SLAM 建图阶段手动 `teleop_keyboard` 跑图的步骤，对 `real.world` 这类大世界尤其有用。

它做的事情：

1. 订阅 `/map`（来自 `slam_toolbox`）
2. 用 numpy 检测前沿（free cell 紧邻 unknown cell）
3. 用 TF 取机器人在 `map` 坐标系下的位姿
4. 按前沿簇选择目标，并把目标回退到已知 free 区
5. 通过 `NavigateToPose` action 发目标，等待结果
6. 目标成功则把该前沿记为已访问，失败/超时/卡死则加入黑名单，避免反复发同一片目标
7. 循环直到没有前沿（地图探索完成）或总时长预算耗尽

它复用了 `mission_runner.py` 的范式：同样是自驱动 Node（无顶层 spin），用 `ActionClient(self, NavigateToPose, ...)`，并用 `spin_until_future_complete` / `spin_once` 推进。

这个节点不直接发 `/cmd_vel`，也不发 `/initialpose`（SLAM 模式下不需要）。它只负责发 `NavigateToPose` 目标，真正的运动控制仍由 Nav2 完成。

主要参数（均有默认值，一般不需要改）：

- `map_topic` / `action_name` / `map_frame` / `base_frame`
- `goal_timeout_sec`（单目标超时，默认 90s）
- `explore_timeout_sec`（总探索预算，默认 1800s）
- `min_frontier_size`（少于该数量的前沿视为探索完成，默认 5）
- `frontier_bin_size_m`（前沿聚类桶大小，默认 0.5m）
- `min_cluster_size`（优先选择不少于该数量的前沿桶；没有大桶时回退到小桶，默认 8）
- `frontier_setback_m`（目标沿机器人方向回退进已知 free 区，默认 0.5m）
- `min_goal_distance_m`（过近的目标忽略，默认 0.5m）
- `blacklist_radius_m`（失败/卡死目标的屏蔽半径，默认 1.0m）
- `visited_radius_m`（已成功到达前沿的屏蔽半径，默认 0.8m）
- `stuck_window_sec` / `stuck_threshold_m`（卡死看门狗：N 秒内位移不足阈值则取消当前目标，默认 10s / 0.1m）
- `post_goal_settle_sec`（目标结束后等待新地图刷新的时间，默认 1s）

> 抗「原地转/重复目标」设计：目标选在**前沿簇心**而非单格（避免边界抖动产生连续微目标），并回退进已知空地（不贴 unknown 边界）；成功到达的前沿会按 `visited_radius_m` 记为已访问，失败/卡死目标会按 `blacklist_radius_m` 屏蔽；执行期间有**卡死看门狗**，机器人原地转超过 `stuck_window_sec` 就判定 `stuck` 取消，不再傻等满整个 `goal_timeout_sec`。空旷处仍转得久时，可调大 `min_cluster_size`、调大 `visited_radius_m` 或调小 `stuck_window_sec`。

输出文件位置：

- `/tmp/nav2_lab_results/*_explore.csv`

## 4. 任务配置

当前任务配置文件：

- `nav2_lab_missions/config/simple_room_mission.yaml`（基线，对应 `simple_room` 世界/地图）
- `nav2_lab_missions/config/real_world_mission.yaml`（对应 `real` 世界/地图，尺度大、目标点跨度达到几十米）

当前字段包括：

- `frame_id`
- `default_timeout_sec`
- `retry_count`
- `publish_initial_pose`（可选，默认 `true`）
- `wait_for_localization`（可选，默认 `true`）
- `initial_pose`
- `goals`

`simple_room_mission` 默认任务点包括：

1. `east_side`
2. `north_west`
3. `home`

`real_world_mission` 任务点跨度更大，包含 `north_station`、`east_service_road`、`south_service_road`、`west_service_road`、`home`，`default_timeout_sec` 提到 240s。

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

## 6. 启动方法

### 6.1 编译

```bash
cd /path/to/nav2_ws
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
ros2 launch nav2_lab_bringup sim.launch.py world:=real
```

`sim.launch.py` 启动时会自动把以下路径加入 `GAZEBO_MODEL_PATH`：TurtleBot3 模型目录、`~/gazebo_models`（本地自定义模型）、`/usr/share/gazebo-11/models`。像 `real.world` 这类带自定义模型的场景，把对应模型放到 `~/gazebo_models` 下即可被识别。

### 6.3 新世界标准流程

> 这是**手动**建图流程（teleop 跑图）。如果不想手动遥控，尤其是 `real.world` 这种大世界，可以直接用 §6.6 的自动探索建图，让机器人自己跑满地图。

以 `real.world` 为例，先启动仿真：

```bash
ros2 launch nav2_lab_bringup sim.launch.py world:=real
```

另开终端启动键盘控制：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

另开终端启动 SLAM：

```bash
ros2 launch nav2_lab_bringup slam.launch.py
```

建图完成后保存地图。在工作区根目录执行即可，`-f` 后面不要写扩展名，命令会生成 `.yaml` 和 `.pgm`：

```bash
ros2 run nav2_map_server map_saver_cli -f nav2_lab_bringup/maps/real
```

保存后停止 SLAM，保持 Gazebo 仿真继续运行，然后用同名地图启动导航：

```bash
ros2 launch nav2_lab_bringup navigation.launch.py map:=real
```

约定：

- `world:=real` 会解析为 `nav2_lab_worlds/worlds/real.world`
- `map:=real` 会解析为 `nav2_lab_bringup/maps/real.yaml`
- 如果 `world` 和 `map` 名字不同，需要显式传对应的名字或绝对路径

### 6.4 自动任务入口

```bash
ros2 launch nav2_lab_bringup lab.launch.py run_mission:=true
```

指定世界和地图：

```bash
ros2 launch nav2_lab_bringup lab.launch.py world:=real map:=real run_mission:=true
```

如果任务点不是默认 `simple_room_mission.yaml`，还需要传对应的任务名：

```bash
ros2 launch nav2_lab_bringup lab.launch.py world:=real map:=real mission:=real_world_mission run_mission:=true
```

约定：

- `mission:=real_world_mission` 会解析为 `nav2_lab_missions/config/real_world_mission.yaml`
- `mission_file:=其他任务名` 只作为高级覆盖参数，一般不需要使用；它也支持 `~` 或绝对路径

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

### 6.6 自动探索建图

`explore.launch.py` 一条命令拉起「仿真 + SLAM + Nav2 + 探索节点」，机器人会自己朝地图前沿走，把地图跑满，省掉手动 teleop。适合 `real.world` 这种大世界。

```bash
ros2 launch nav2_lab_bringup explore.launch.py world:=real
```

它会做的事：

1. 启动 Gazebo（`world:=real`）
2. 以 SLAM 模式启动 Nav2（`slam_toolbox` 发布 `/map`，AMCL / map_server 不启动）
3. 延迟约 12s 启动 `explore_runner`，开始前沿探索并持续建图

RViz 会同步显示地图生长、代价地图和路径。探索过程会写入 `/tmp/nav2_lab_results/*_explore.csv`，记录每个目标的状态和耗时。

常用参数（可选）：

```bash
ros2 launch nav2_lab_bringup explore.launch.py world:=real explore_timeout_sec:=3600 goal_timeout_sec:=120
```

无桌面/远程验证时可关闭图形界面：

```bash
ros2 launch nav2_lab_bringup explore.launch.py world:=real use_rviz:=false use_gzclient:=false
```

约定：

- `world:=real` 解析为 `nav2_lab_worlds/worlds/real.world`，与 §6.3 一致
- `explore_timeout_sec`：总探索预算（默认 1800s）
- `goal_timeout_sec`：单目标超时（默认 90s）
- `visited_radius_m`：成功到达后屏蔽同一片前沿的半径（默认 0.8m）
- `blacklist_radius_m`：失败/卡死后屏蔽前沿的半径（默认 1.0m）
- `min_cluster_size` / `frontier_bin_size_m`：前沿簇优先级与聚类粗细
- `stuck_window_sec` / `stuck_threshold_m`：原地转或卡死判定

探索完成（日志出现 `No more frontiers detected`）后，在另一个终端保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f nav2_lab_bringup/maps/real
```

注意：

- 当前 SLAM 走的是 `slam_toolbox` 的 **sync** 变体（由 `navigation.launch.py` 的 `slam:=True` 提供）。对仿真 TurtleBot3 足够，建图质量受影响时再考虑切到 `online_async`。
- 探索节点靠 TF 取 `map → base_link` 位姿，靠 `/map` 检测前沿；两者都来自 SLAM 栈，所以必须用 SLAM 模式启动。

### 6.7 基线结果统计

`mission_runner` 每次运行会在 `/tmp/nav2_lab_results` 下生成 `*_mission.csv`。可以用内置统计命令汇总一次或多次运行结果：

```bash
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results
```

如果要把 `simple_room` 当作回归门槛，可以加上 `--require-success`。只要有任意一次 mission 的最终目标状态不是 `succeeded`，命令就会返回非 0：

```bash
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results --require-success
```

项目内置了 `simple_room` 基线阈值。连续跑完多次 `simple_room_mission` 后，用下面的命令可以直接判定是否符合当前基线：

```bash
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results --baseline simple_room
```

推荐的基线固化方式：

1. 清空或单独保存旧的 `/tmp/nav2_lab_results/*_mission.csv`
2. 连续跑 5 到 10 次 `simple_room_mission`
3. 用 `mission_stats --baseline simple_room` 检查成功率、重试次数和目标耗时
4. 后续修改 launch、参数或算法前后，都先比较这套结果

## 7. 已经发现并修过的问题

### 7.1 任务发送过早

问题：

- `mission_runner` 早于 Nav2 完全激活就发送目标
- `NavigateToPose` 会直接 reject

修复：

- 等待 `/amcl_pose`
- 再额外等待一段激活稳定时间

### 7.2 初始位姿竞争条件

问题：

- `/initialpose` 可能发得太早
- AMCL 还没完成订阅发现，导致初始位姿丢失

修复：

- 先等待 `/initialpose` 订阅端就绪
- 在短时间内重复发布初始位姿

### 7.3 RViz 过于简化

问题：

- 自定义 RViz 配置太薄
- 手动调试时地图和目标交互不稳定

修复：

- 默认切到 Nav2 官方 `nav2_default_view.rviz`

## 8. 当前限制

虽然这一版已经能跑通，但它还是一个刻意保持简单的基线版本。

当前限制包括：

- 目前有两套地图：`simple_room`和 `real`
- 任务统计还比较基础
- 还没有参数实验矩阵
- 还没有自定义 planner / controller / BT 插件
- 还没有动态障碍物实验

## 9. 后续开发方向

### 9.1 先稳住基线

- 把 `simple_room` 作为标准参考场景
- 固化预期 RViz 现象和 CSV 输出样例

### 9.2 加实验变体

- 在 `real` 世界之外，再补一两种结构差异明显的场景（如窄走廊、多房间），各自配齐 world / map / mission
- 为每个场景写专用 mission YAML
- 对比不同参数下的成功率、路径质量、耗时

### 9.3 加参数调优资产

先建立多套参数版本，重点观察：

- `inflation_radius`
- `controller_frequency`
- `max_vel_x`
- `xy_goal_tolerance`
- local/global costmap 尺寸

### 9.4 加诊断与统计

- 统计 recovery 触发次数
- 汇总任务总耗时
- 汇总重试次数
- 更明确记录失败原因

### 9.5 再进入真正的扩展开发

当这条基线足够稳定后，最值得做的工程化进阶方向是：

- 自定义 BT XML
- 自定义 recovery behavior
- 自定义 costmap layer
- 简单自定义 planner plugin

## 10. 实用总结

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
