# nav2_lab

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-2C2C2C?logo=gazebo)
![Nav2](https://img.shields.io/badge/Nav2-Navigation2-22314E)
![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Status](https://img.shields.io/badge/status-experiment-orange)

> 一个以仿真为起点的 **Nav2 实验/学习平台**：拉起仿真、自动导航多目标、记录结果，并自带三个自定义 Nav2 插件（recovery / planner / costmap layer）。

`nav2_lab` 的目标不是"功能多"，而是 **结构清楚、链路可复现、便于扩展**——从基线导航一路走到自带可切换的自定义插件，且每个扩展都有验证证据。

---

## ✨ Features

- 🗺 **三套场景**：`simple_room` / `real` / `goal_patience`
- 🤖 **自动任务执行**：顺序下发 `NavigateToPose`，输出 mission / telemetry CSV
- 📈 **统计与对比**：`mission_stats` 支持基线判定、多批 `--compare`、`--details` 下钻
- 🧭 **自动探索建图**：前沿探索替掉手动 teleop，适合 `real` 这类大世界
- 🧩 **三个自定义 Nav2 插件**（均已编译并验证）：
  - `ObserveSpin` — 智能旋转 recovery（扫障入图）
  - `StraightLine` — 直线全局规划器（教学）
  - `MarkerLayer` — 固定点代价衰减 costmap layer
- 🔁 **一键基线脚本**：连续跑 N 次 + 自动归档 + 基线判定

技术栈：ROS 2 Humble · Gazebo Classic · TurtleBot3 Waffle · Nav2 · AMCL · SLAM Toolbox

---

## 📁 仓库结构

```text
nav2_ws/
├── nav2_lab_bringup/          # 顶层启动（ament_cmake, 数据包）
│   ├── launch/                # sim / slam / navigation / lab / explore
│   ├── params/                # nav2_params.yaml（插件注册在这里）
│   ├── maps/                  # simple_room / real
│   ├── behavior_trees/        # goal_patience / recovery / straight_line
│   └── rviz/
├── nav2_lab_worlds/           # Gazebo 世界（ament_cmake, 数据包）
│   └── worlds/                # simple_room / real / goal_patience
├── nav2_lab_missions/         # 自动任务节点 + 统计（ament_python）
│   ├── nav2_lab_missions/     # mission_runner / logger / explore_runner / mission_stats
│   ├── config/                # *_mission.yaml
│   ├── baselines/             # simple_room 基线阈值
│   └── test/
├── nav2_lab_behaviors/        # ObserveSpin（C++ recovery 插件）
├── nav2_lab_planners/         # StraightLine（C++ planner 插件）
├── nav2_lab_costmaps/         # MarkerLayer（C++ costmap layer 插件）
├── run_simple_room_baseline.sh
├── run_goal_patience_experiment.sh
└── README.md
```

---

## 🚀 Quick Start

### 前置依赖

ROS 2 Humble + Nav2 + Gazebo Classic + TurtleBot3 功能包（`turtlebot3_gazebo` / `turtlebot3_teleop` 等）已安装，且已 `source /opt/ros/humble/setup.bash`。

```bash
export TURTLEBOT3_MODEL=waffle
```

### 编译

```bash
cd /path/to/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 跑第一个任务

```bash
# 一键拉起仿真 + Nav2 + 自动任务（simple_room，默认 Navfn）
ros2 launch nav2_lab_bringup lab.launch.py run_mission:=true
```

结果落在 `/tmp/nav2_lab_results/*_mission.csv`。

---

## 🎮 启动入口

| 目的 | 命令 |
| --- | --- |
| 仅仿真 | `ros2 launch nav2_lab_bringup sim.launch.py` |
| 自动任务 | `ros2 launch nav2_lab_bringup lab.launch.py run_mission:=true` |
| 自动探索建图 | `ros2 launch nav2_lab_bringup explore.launch.py world:=real` |
| 手动 RViz 调试 | `ros2 launch nav2_lab_bringup lab.launch.py run_mission:=false` |

### 手动建图（新世界标准流程）

> 这是**手动 teleop 跑图**流程。如果不想手动遥控，尤其 `real.world` 这种大世界，直接用上面的 **自动探索建图** 让机器人自己跑满地图。

以 `real.world` 为例。

**1. 启动仿真**

```bash
ros2 launch nav2_lab_bringup sim.launch.py world:=real
```

**2. 另开终端，启动键盘控制**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**3. 再开终端，启动 SLAM**

```bash
ros2 launch nav2_lab_bringup slam.launch.py
```

**4. 建图完成后保存地图**（在工作区根目录执行，`-f` 后不带扩展名，会生成 `.yaml` + `.pgm`）

```bash
ros2 run nav2_map_server map_saver_cli -f nav2_lab_bringup/maps/real
```

**5. 停止 SLAM，保持仿真，用同名地图启动导航**

```bash
ros2 launch nav2_lab_bringup navigation.launch.py map:=real
```

**命名约定**

- `world:=real` → `nav2_lab_worlds/worlds/real.world`
- `map:=real` → `nav2_lab_bringup/maps/real.yaml`
- 若 `world` 与 `map` 名字不同，需显式传对应名字或绝对路径

### 自动任务常用参数

```bash
ros2 launch nav2_lab_bringup lab.launch.py \
  world:=real map:=real mission:=real_world_mission \
  run_mission:=true \
  bt_xml:=nav2_lab_recovery          # 可选：切到自定义 recovery 树
```

`lab.launch.py` 参数：`world` `map` `model` `x_pose/y_pose/yaw` `mission` `bt_xml` `params_file` `slam` `run_mission` `shutdown_on_mission_complete` `use_rviz` `use_sim_time`。名字（`world`/`map`/`mission`/`bt_xml`）会自动解析到对应目录。

> ⚠️ **手动调试坑**：`run_mission:=false` 时没人发 `/initialpose`，必须用 RViz 的 **2D Pose Estimate** 设初始位姿，否则 AMCL 不收敛、nav 栈无法激活。

---

## 🧩 自定义插件

三个插件都走 pluginlib + 对应 Nav2 接口，与内置插件并存、可切换：

| 插件 | base class | 接入点 | 切换方式 | 验证 |
| --- | --- | --- | --- | --- |
| **ObserveSpin** | `nav2_core::Behavior` | `behavior_server` | `bt_xml:=nav2_lab_recovery` | ✅ 加载 + 运行时旋转探测 |
| **StraightLine** | `nav2_core::GlobalPlanner` | `planner_server` | `bt_xml:=nav2_lab_straight_line` | ✅ 加载 + 直线 path + mission |
| **MarkerLayer** | `nav2_costmap_2d::Layer` | `global_costmap` | params 默认启用 | ✅ 加载 + 影响 Navfn 路径 |

### ObserveSpin（recovery）

默认 recovery 遇障先清空 costmap，在"未建图障碍"场景会陷入"擦图 → 重规划 → 撞墙"死循环。ObserveSpin 失败时**朝 local costmap 更空的一侧旋转**，让激光把未建图障碍扫进 costmap，使下次重规划绕开。继承 `TimedBehavior<SpinAction>`（复用 Spin action），`onRun` 探测左右选向、`onCycleUpdate` 梯形角速度控制。

参数（`behavior_server.observe_spin.*`）：`candidate_angle`(1.57) · `probe_angle`(0.785) · `max_rotational_vel`(1.0) · `min_rotational_vel`(0.4) · `rotational_acc_lim`(3.2) · `simulate_ahead_time`(2.0)

> 实测：goal_patience 场景默认 BT 已能靠实时 costmap 绕开障碍，recovery 触发不多；ObserveSpin 的价值是"智能 recovery"样本与后续实验基础。

### StraightLine（planner）

start→goal 连一条 Bresenham 直线，逐 cell 查 costmap，无致命障碍则输出直线 path，穿障则返回**空 path**。继承 `nav2_core::GlobalPlanner`，`createPlan` 用 `nav2_util::LineIterator` + `Costmap2D::getCost`/`mapToWorld`。Navfn(GridBased) 保留为默认。

参数（`planner_server.LabStraightLine.*`）：`obstacle_cost_threshold`(253.0) · `allow_unknown`(false) · `sample_step`(1)

> 实测：直线特征铁证（返回点严格共线），simple_room 3 goal 全 succeeded。直线遇障即败，只适合干净场景或教学对比。

### MarkerLayer（costmap layer）

在参数化固定点 (x,y) 周围写一圈**线性衰减 cost**（中心 `peak_cost`、边缘 0），用 `std::max` 合并进 master grid（不降低既有 cost）。继承 `nav2_costmap_2d::Layer`，仅依赖参数、无需订阅/TF。默认已加到 `global_costmap` 的 plugins（`static_layer` 之后、`inflation_layer` 之前），改 params 的 `point_x`/`point_y`/`radius` 即可移动标记。

参数（`global_costmap.lab_marker_layer.*`）：`enabled`(true) · `point_x`(0.0) · `point_y`(0.0) · `radius`(0.5) · `peak_cost`(254)

> 实测：marker 压在 north_west 路径上时，Navfn 绕开（mission 仍成功）。

---

## 📊 任务、基线与统计

**任务配置**（`nav2_lab_missions/config/*.yaml`）：`simple_room_mission` / `real_world_mission` / `goal_patience_mission`。字段：`frame_id` `default_timeout_sec` `retry_count` `publish_initial_pose` `wait_for_localization` `initial_pose` `goals`。

**输出**：`mission_runner` 每次运行在 `/tmp/nav2_lab_results/` 生成 `*_mission.csv`（含 `recovery_count`，来自 Nav2 feedback 的 `number_of_recoveries`）；`mission_logger` 生成 `*_telemetry.csv`。

```bash
# 汇总一次或多次
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results
# 设为回归门槛
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results --require-success
# 用内置 simple_room 基线判定
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results --baseline simple_room
# 两批结果对比（如默认 BT vs 自定义 BT）
ros2 run nav2_lab_missions mission_stats --compare <archive_a> <dir_b> --labels a b
# 定位具体 run/goal/attempt 的 timeout/recovery
ros2 run nav2_lab_missions mission_stats /tmp/nav2_lab_results --details
```

**一键基线脚本**（自动归档旧结果到 `/tmp/nav2_lab_results/archive/`，写 `metadata.env`）：

```bash
./run_simple_room_baseline.sh 5          # simple_room 连跑 5 次 + 基线判定
./run_goal_patience_experiment.sh 5      # goal_patience 连跑 5 次（可配 NAV2_LAB_BT_XML / NAV2_LAB_EXPERIMENT）
```

---

## 🔁 数据流

```text
Gazebo TurtleBot3
  → /scan /odom /tf
  → map_server 发布 /map
  → mission_runner 发 /initialpose
  → AMCL 消费 /map + /scan，发 /amcl_pose 和 map→odom
  → mission_runner 发 NavigateToPose goal
  → bt_navigator 编排（planner_server 规划 / controller_server 控制 / behavior_server recovery）
  → /cmd_vel → Gazebo 执行
```

三个自定义插件在这条链路上的位置：

- **StraightLine**（planner_server）：替代 Navfn 生成 global path
- **MarkerLayer**（global_costmap）：在 costmap 上加 cost，影响 planner 规划
- **ObserveSpin**（behavior_server）：导航失败时被 BT 的 RecoveryActions 调用

---

## 🛠 已知问题与修复

| 问题 | 修复 |
| --- | --- |
| mission 过早发目标 → `NavigateToPose` reject | 等 `/amcl_pose` + 额外激活延时 |
| `/initialpose` 发太早、订阅端没就绪 | 等订阅端就绪 + 短时间内重复发布 |
| 自定义 RViz 配置太薄、交互不稳 | 默认切到官方 `nav2_default_view.rviz` |
| 残留 `gzserver` 进程导致新仿真启动失败 (`gzserver exit 255`) | 启动前 `pkill -f gzserver` 清掉孤儿进程 |

---

## 🗺 Roadmap

**当前限制**

- 参数实验矩阵尚未建立
- 三个插件已跑通链路，但缺系统的参数 / 场景对比实验
- 没有动态障碍实验

**后续可做**

- 参数调优资产（`inflation_radius` / `controller_frequency` / `max_vel_x` / `xy_goal_tolerance` 等多套版本对比）
- 补结构差异明显的场景（窄走廊、多房间），各自配齐 world / map / mission
- 动态障碍实验
- 三个插件上的系统对比实验（StraightLine vs Navfn 跨场景成功率、MarkerLayer 不同位置的规划影响、ObserveSpin 在真正卡住场景的救场效果）

---

## 📄 License

Apache-2.0。详见 [LICENSE](LICENSE)。
