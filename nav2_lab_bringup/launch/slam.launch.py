import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_lab_bringup')
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
            description='Parameter file used by SLAM Toolbox.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'nav2_lab.rviz'),
            description='RViz config file.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'slam_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
