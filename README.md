# nav2_lab

基于 ROS 2 Humble、Nav2、Gazebo Classic 和 TurtleBot3 的导航学习与插件开发工作区。

这个项目只保留三条主线：

1. 给新的 Gazebo 世界建立地图。
2. 使用已有地图一键启动仿真、Nav2 和 RViz，手动设置目标点。
3. 在官方 Nav2 基线上逐步接入自己的 planner、behavior 和 costmap layer。

默认启动只使用官方 Nav2 插件。仓库已有的三个自定义插件必须通过启动参数显式启用。

## 目录结构

```text
nav2_ws/
├── nav2_lab_bringup/          # 启动文件、Nav2 参数、地图、BT、RViz
│   ├── launch/
│   │   ├── sim.launch.py      # 只启动 Gazebo 和机器人
│   │   ├── slam.launch.py     # 只启动 SLAM Toolbox
│   │   ├── mapping.launch.py  # Gazebo + SLAM + RViz
│   │   ├── navigation.launch.py
│   │   ├── lab.launch.py      # Gazebo + Nav2 + RViz
│   │   └── explore.launch.py  # 大世界自动探索建图
│   ├── maps/
│   ├── params/nav2_params.yaml
│   └── behavior_trees/
├── nav2_lab_worlds/           # Gazebo 世界
├── nav2_lab_explorer/         # 前沿自动探索节点
├── nav2_lab_planners/         # StraightLinePlanner
├── nav2_lab_behaviors/        # ObserveSpin
└── nav2_lab_costmaps/         # MarkerLayer
```

项目不包含自动导航任务、任务 CSV、实验基线或批量实验脚本。导航目标由用户在 RViz 中手动设置。

## 环境与编译

需要安装：

- ROS 2 Humble
- Nav2
- Gazebo Classic / gazebo_ros
- TurtleBot3 Gazebo 与 teleop
- SLAM Toolbox

```bash
cd /path/to/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每个新终端都需要 source：

```bash
source /opt/ros/humble/setup.bash
source /path/to/nav2_ws/install/setup.bash
```

## 新世界标准流程

下面以新世界 `warehouse` 为例。推荐让世界、地图 YAML 和地图图像使用相同的基本名称。

### 1. 添加世界

将世界文件放到：

```text
nav2_lab_worlds/worlds/warehouse.world
```

重新编译并 source：

```bash
colcon build --symlink-install --packages-select nav2_lab_worlds nav2_lab_bringup
source install/setup.bash
```

### 2. 启动建图环境

一条命令启动 Gazebo、机器人、SLAM Toolbox 和 RViz：

```bash
ros2 launch nav2_lab_bringup mapping.launch.py world:=warehouse
```

如果默认出生点不在空闲区域，可指定出生位姿：

```bash
ros2 launch nav2_lab_bringup mapping.launch.py \
  world:=warehouse \
  x_pose:=1.0 y_pose:=2.0 yaw:=1.57
```

### 3. 手动移动机器人完成建图

另开终端：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

移动时应覆盖所有可通行区域，并尽量从不同方向重复观察走廊、门口和大型障碍物，以便 SLAM 完成回环和地图闭合。

### 4. 保存地图

在工作区根目录执行：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f nav2_lab_bringup/maps/warehouse
```

应生成：

```text
nav2_lab_bringup/maps/warehouse.yaml
nav2_lab_bringup/maps/warehouse.pgm
```

保存后重新编译 `nav2_lab_bringup`，确保地图进入安装空间：

```bash
colcon build --symlink-install --packages-select nav2_lab_bringup
source install/setup.bash
```

### 5. 使用地图一键启动导航

结束建图进程后执行：

```bash
ros2 launch nav2_lab_bringup lab.launch.py \
  world:=warehouse \
  map:=warehouse
```

在 RViz 中：

1. 使用 **2D Pose Estimate** 设置机器人初始位姿。
2. 等待地图、激光和代价地图正常显示。
3. 使用 **Nav2 Goal** 设置目标点。

`world` 与 `map` 是两个独立参数。使用同名只是推荐约定，也可以显式组合不同的世界和地图。

## 大世界自动探索

对于不适合手动 teleop 的大世界，保留了前沿自动探索：

```bash
ros2 launch nav2_lab_bringup explore.launch.py world:=warehouse
```

该入口会启动 Gazebo、SLAM 模式的完整 Nav2 栈和 `nav2_lab_explorer`。探索结束后仍需手动保存地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f nav2_lab_bringup/maps/warehouse
```

常用探索参数：

```bash
ros2 launch nav2_lab_bringup explore.launch.py \
  world:=warehouse \
  explore_timeout_sec:=1800.0 \
  goal_timeout_sec:=90.0 \
  min_frontier_size:=5 \
  frontier_setback_m:=0.5
```

自动探索属于辅助功能。不同地图尺寸和通道宽度可能需要调整前沿聚类、黑名单半径和卡住检测参数。

## 启动入口

| 目的 | 命令 |
| --- | --- |
| 只启动仿真 | `ros2 launch nav2_lab_bringup sim.launch.py world:=simple_room` |
| 手动 SLAM 建图 | `ros2 launch nav2_lab_bringup mapping.launch.py world:=simple_room` |
| 自动探索建图 | `ros2 launch nav2_lab_bringup explore.launch.py world:=real` |
| 已有地图导航 | `ros2 launch nav2_lab_bringup lab.launch.py world:=simple_room map:=simple_room` |
| 只启动导航栈 | `ros2 launch nav2_lab_bringup navigation.launch.py map:=simple_room` |

`lab.launch.py` 常用参数：

- `world`：世界名称或绝对 `.world` 路径。
- `map`：地图名称或绝对 `.yaml` 路径。
- `model`：TurtleBot3 型号，默认 `waffle`。
- `x_pose`、`y_pose`、`yaw`：Gazebo 中的机器人出生位姿。
- `params_file`：Nav2 参数文件。
- `bt_xml`：NavigateToPose 行为树名称或绝对路径。
- `use_rviz`：是否启动 RViz。
- `use_gzclient`：是否启动 Gazebo GUI。

## 官方基线与自定义插件

默认命令：

```bash
ros2 launch nav2_lab_bringup lab.launch.py
```

默认使用项目配置中的官方插件：

- Navfn 全局规划器
- DWB 局部控制器
- 官方 Spin、BackUp、Wait 等恢复行为
- Static、Obstacle、Voxel、Inflation costmap layers

已有三个自定义插件默认不加载：

| 插件 | Nav2 接口 | 启用参数 |
| --- | --- | --- |
| `StraightLinePlanner` | `nav2_core::GlobalPlanner` | `enable_straight_line_planner:=true` |
| `ObserveSpin` | `nav2_core::Behavior` | `enable_observe_spin:=true` |
| `MarkerLayer` | `nav2_costmap_2d::Layer` | `enable_marker_layer:=true` |

只启用一个插件：

```bash
ros2 launch nav2_lab_bringup lab.launch.py \
  enable_observe_spin:=true
```

同时启用三个插件：

```bash
ros2 launch nav2_lab_bringup lab.launch.py \
  enable_straight_line_planner:=true \
  enable_observe_spin:=true \
  enable_marker_layer:=true
```

启动器会按开关加载插件并选择匹配的行为树。需要直接控制 BT 时仍可使用：

```bash
ros2 launch nav2_lab_bringup lab.launch.py \
  bt_xml:=nav2_lab_recovery
```

三个插件目前主要用于学习 Nav2 pluginlib 接入方式：

- `StraightLinePlanner` 只能生成无障碍直线，不会主动绕障。
- `ObserveSpin` 只在 BT 进入恢复流程时执行。
- `MarkerLayer` 在固定坐标周围写入代价，启用前需按地图调整坐标。

## 调参与新增插件

统一参数文件是：

```text
nav2_lab_bringup/params/nav2_params.yaml
```

调参时可以直接修改它，也可以复制一份后从命令行传入：

```bash
ros2 launch nav2_lab_bringup lab.launch.py \
  params_file:=/absolute/path/to/my_nav2_params.yaml
```

新增插件的一般步骤：

1. 在独立 ROS 2 包中实现对应 Nav2 接口。
2. 通过 pluginlib XML、CMake 和 `package.xml` 导出插件。
3. 在参数文件对应服务器下增加插件 ID 和参数。
4. 如果是 planner 或 behavior，在 BT 中引用相应 ID/action server。
5. 在 `navigation.launch.py` 增加启动参数或选择逻辑。
6. 从 `lab.launch.py` 将该参数透传给 `navigation.launch.py`。

推荐保持“默认官方、显式启用自定义”的原则，这样每个算法都能和官方基线直接对照。

## 数据流

导航模式：

```text
Gazebo → /scan /odom /tf
       → map_server + AMCL
       → planner_server + controller_server + behavior_server
       → /cmd_vel → Gazebo
```

建图模式：

```text
Gazebo → /scan /odom /tf
       → SLAM Toolbox → /map + map→odom
       → map_saver_cli → .yaml + .pgm
```

## License

Apache-2.0
