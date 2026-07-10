import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_map(map_name, package_share):
    expanded = os.path.expanduser(map_name)
    if os.path.isabs(expanded):
        return expanded

    candidate = expanded
    if not candidate.endswith('.yaml'):
        candidate = f'{candidate}.yaml'

    installed_map = os.path.join(package_share, 'maps', candidate)
    if os.path.exists(installed_map):
        return installed_map

    package_xml = os.path.join(package_share, 'package.xml')
    if os.path.islink(package_xml):
        source_package = os.path.dirname(os.path.realpath(package_xml))
        source_map = os.path.join(source_package, 'maps', candidate)
        if os.path.exists(source_map):
            return source_map

    return installed_map


def _resolve_behavior_tree(value, package_share):
    expanded = os.path.expanduser(value)
    if os.path.isabs(expanded):
        return expanded

    candidate = expanded
    if not candidate.endswith('.xml'):
        candidate = f'{candidate}.xml'

    installed_tree = os.path.join(package_share, 'behavior_trees', candidate)
    if os.path.exists(installed_tree):
        return installed_tree

    package_xml = os.path.join(package_share, 'package.xml')
    if os.path.islink(package_xml):
        source_package = os.path.dirname(os.path.realpath(package_xml))
        source_tree = os.path.join(source_package, 'behavior_trees', candidate)
        if os.path.exists(source_tree):
            return source_tree

    return installed_tree


def _params_file_with_bt_override(params_file, bt_xml, package_share):
    expanded_params_file = os.path.expanduser(params_file)
    if not bt_xml:
        return expanded_params_file

    bt_xml_file = _resolve_behavior_tree(bt_xml, package_share)
    if not os.path.exists(bt_xml_file):
        raise FileNotFoundError(f'BT XML file not found: {bt_xml_file}')

    with open(expanded_params_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}

    bt_params = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
    bt_params.pop('default_bt_xml_filename', None)
    bt_params['default_nav_to_pose_bt_xml'] = bt_xml_file

    temp_file = tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        prefix='nav2_lab_nav2_params_',
        suffix='.yaml',
        delete=False,
    )
    with temp_file:
        yaml.safe_dump(params, temp_file, sort_keys=False)
    return temp_file.name


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('nav2_lab_bringup')
    nav2_bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    map_file = _resolve_map(LaunchConfiguration('map').perform(context), pkg_share)
    params_file = _params_file_with_bt_override(
        LaunchConfiguration('params_file').perform(context),
        LaunchConfiguration('bt_xml').perform(context),
        pkg_share,
    )
    rviz_config = LaunchConfiguration('rviz_config')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'slam': slam,
                'autostart': 'True',
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
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_lab_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='simple_room',
            description='Map name under nav2_lab_bringup/maps or absolute .yaml path.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
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
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'),
            description='RViz config file.',
        ),
        DeclareLaunchArgument('slam', default_value='False', description='Run Nav2 with SLAM instead of AMCL.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time.'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz.'),
        OpaqueFunction(function=launch_setup),
    ])
