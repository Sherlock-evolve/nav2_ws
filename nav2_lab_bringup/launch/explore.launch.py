"""Autonomous exploration bringup.

Single entry point for frontier-based mapping: launches the Gazebo simulation,
the Nav2 stack in SLAM mode (slam_toolbox publishes /map, no AMCL / map_server),
and the ``explore_runner`` node that drives the robot toward frontiers until the
map is fully explored. This replaces the manual ``teleop_keyboard`` step.

Typical usage:

    ros2 launch nav2_lab_bringup explore.launch.py world:=real

When exploration finishes (explore_runner reports no more frontiers), save the
map from another terminal:

    ros2 run nav2_map_server map_saver_cli -f nav2_lab_bringup/maps/real
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('nav2_lab_bringup')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gzclient = LaunchConfiguration('use_gzclient')
    explore_timeout = LaunchConfiguration('explore_timeout_sec')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    min_frontier_size = LaunchConfiguration('min_frontier_size')
    min_goal_distance = LaunchConfiguration('min_goal_distance_m')
    frontier_bin_size = LaunchConfiguration('frontier_bin_size_m')
    min_cluster_size = LaunchConfiguration('min_cluster_size')
    frontier_setback = LaunchConfiguration('frontier_setback_m')
    blacklist_radius = LaunchConfiguration('blacklist_radius_m')
    visited_radius = LaunchConfiguration('visited_radius_m')
    stuck_window = LaunchConfiguration('stuck_window_sec')
    stuck_threshold = LaunchConfiguration('stuck_threshold_m')
    post_goal_settle = LaunchConfiguration('post_goal_settle_sec')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'sim.launch.py')),
            launch_arguments={
                'world': world,
                'model': model,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'yaw': yaw,
                'use_sim_time': use_sim_time,
                'use_gzclient': use_gzclient,
            }.items(),
        ),
        # Nav2 in SLAM mode: slam_toolbox publishes /map + map->odom; the full
        # planner/controller/BT stack comes up so explore_runner can send
        # NavigateToPose goals. AMCL and map_server are NOT started. The map
        # arg is therefore unused here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'params_file': params_file,
                'bt_xml': bt_xml,
                'slam': 'True',
                'use_sim_time': use_sim_time,
                'use_rviz': use_rviz,
            }.items(),
        ),
        # Let the Nav2 stack activate before the explorer starts sending goals.
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='nav2_lab_missions',
                    executable='explore_runner',
                    name='explore_runner',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'explore_timeout_sec': ParameterValue(explore_timeout, value_type=float),
                        'goal_timeout_sec': ParameterValue(goal_timeout, value_type=float),
                        'min_frontier_size': ParameterValue(min_frontier_size, value_type=int),
                        'min_goal_distance_m': ParameterValue(min_goal_distance, value_type=float),
                        'frontier_bin_size_m': ParameterValue(frontier_bin_size, value_type=float),
                        'min_cluster_size': ParameterValue(min_cluster_size, value_type=int),
                        'frontier_setback_m': ParameterValue(frontier_setback, value_type=float),
                        'blacklist_radius_m': ParameterValue(blacklist_radius, value_type=float),
                        'visited_radius_m': ParameterValue(visited_radius, value_type=float),
                        'stuck_window_sec': ParameterValue(stuck_window, value_type=float),
                        'stuck_threshold_m': ParameterValue(stuck_threshold, value_type=float),
                        'post_goal_settle_sec': ParameterValue(post_goal_settle, value_type=float),
                    }],
                ),
            ],
        ),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('nav2_lab_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_room', description='World name or absolute .world path.'),
        DeclareLaunchArgument('model', default_value='waffle', description='TurtleBot3 model.'),
        DeclareLaunchArgument('x_pose', default_value='-1.2', description='Initial robot x position.'),
        DeclareLaunchArgument('y_pose', default_value='-1.2', description='Initial robot y position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_share, 'params', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'bt_xml',
            default_value='',
            description=(
                'Optional NavigateToPose BT XML name under '
                'nav2_lab_bringup/behavior_trees or absolute .xml path.'
            ),
        ),
        DeclareLaunchArgument('explore_timeout_sec', default_value='1800.0', description='Overall exploration time budget in seconds.'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='90.0', description='Per-goal timeout in seconds.'),
        DeclareLaunchArgument('min_frontier_size', default_value='5', description='Minimum frontier cell count before treating exploration as complete.'),
        DeclareLaunchArgument('min_goal_distance_m', default_value='0.5', description='Ignore frontier cells closer than this distance.'),
        DeclareLaunchArgument('frontier_bin_size_m', default_value='0.5', description='Coarse frontier bucket size in meters.'),
        DeclareLaunchArgument('min_cluster_size', default_value='8', description='Minimum frontier bucket size preferred as a goal.'),
        DeclareLaunchArgument('frontier_setback_m', default_value='0.5', description='Pull selected frontier goals back into known free space.'),
        DeclareLaunchArgument('blacklist_radius_m', default_value='1.0', description='Radius around failed frontiers to avoid retrying.'),
        DeclareLaunchArgument('visited_radius_m', default_value='0.8', description='Radius around successful frontiers to avoid repeat goals.'),
        DeclareLaunchArgument('stuck_window_sec', default_value='10.0', description='Seconds without translation before canceling a goal.'),
        DeclareLaunchArgument('stuck_threshold_m', default_value='0.1', description='Minimum translation that resets the stuck watchdog.'),
        DeclareLaunchArgument('post_goal_settle_sec', default_value='1.0', description='Wait for a fresh map update after each goal.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument('use_gzclient', default_value='true', description='Start the Gazebo GUI client.'),
        OpaqueFunction(function=launch_setup),
    ])
