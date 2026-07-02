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
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    explore_timeout = LaunchConfiguration('explore_timeout_sec')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')

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
        DeclareLaunchArgument('explore_timeout_sec', default_value='1800.0', description='Overall exploration time budget in seconds.'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='90.0', description='Per-goal timeout in seconds.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        OpaqueFunction(function=launch_setup),
    ])
