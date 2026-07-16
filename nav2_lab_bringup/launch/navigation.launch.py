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


def _as_bool(value, argument_name):
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{argument_name} must be true or false, got: {value}')


def _append_unique(values, item):
    if item not in values:
        values.append(item)


def _insert_before(values, item, before):
    if item in values:
        return
    if before in values:
        values.insert(values.index(before), item)
    else:
        values.append(item)


def _params_file_with_plugin_selection(
    params_file,
    bt_xml,
    package_share,
    enable_straight_line_planner=False,
    enable_observe_spin=False,
    enable_marker_layer=False,
):
    expanded_params_file = os.path.expanduser(params_file)
    selected_bt = bt_xml

    # Preserve the old bt_xml shortcuts: selecting one of the bundled custom
    # trees also loads the plugin(s) referenced by that tree.
    bt_name = os.path.splitext(os.path.basename(bt_xml))[0] if bt_xml else ''
    if bt_name in ('nav2_lab_straight_line', 'nav2_lab_all_plugins'):
        enable_straight_line_planner = True
    if bt_name in ('nav2_lab_recovery', 'nav2_lab_all_plugins'):
        enable_observe_spin = True

    # When only plugin switches are supplied, choose the matching bundled BT.
    if not selected_bt:
        if enable_straight_line_planner and enable_observe_spin:
            selected_bt = 'nav2_lab_all_plugins'
        elif enable_straight_line_planner:
            selected_bt = 'nav2_lab_straight_line'
        elif enable_observe_spin:
            selected_bt = 'nav2_lab_recovery'

    if not any((selected_bt, enable_marker_layer)):
        return expanded_params_file

    with open(expanded_params_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}

    if enable_straight_line_planner:
        planner_params = params.setdefault('planner_server', {}).setdefault(
            'ros__parameters', {}
        )
        planner_plugins = planner_params.setdefault('planner_plugins', [])
        _append_unique(planner_plugins, 'LabStraightLine')

    if enable_observe_spin:
        behavior_params = params.setdefault('behavior_server', {}).setdefault(
            'ros__parameters', {}
        )
        behavior_plugins = behavior_params.setdefault('behavior_plugins', [])
        _append_unique(behavior_plugins, 'observe_spin')

    if enable_marker_layer:
        global_costmap_params = params.setdefault('global_costmap', {}).setdefault(
            'global_costmap', {}
        ).setdefault('ros__parameters', {})
        costmap_plugins = global_costmap_params.setdefault('plugins', [])
        _insert_before(costmap_plugins, 'lab_marker_layer', 'inflation_layer')
        marker_params = global_costmap_params.setdefault('lab_marker_layer', {})
        marker_params['enabled'] = True

    if selected_bt:
        bt_xml_file = _resolve_behavior_tree(selected_bt, package_share)
        if not os.path.exists(bt_xml_file):
            raise FileNotFoundError(f'BT XML file not found: {bt_xml_file}')
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
    params_file = _params_file_with_plugin_selection(
        LaunchConfiguration('params_file').perform(context),
        LaunchConfiguration('bt_xml').perform(context),
        pkg_share,
        enable_straight_line_planner=_as_bool(
            LaunchConfiguration('enable_straight_line_planner').perform(context),
            'enable_straight_line_planner',
        ),
        enable_observe_spin=_as_bool(
            LaunchConfiguration('enable_observe_spin').perform(context),
            'enable_observe_spin',
        ),
        enable_marker_layer=_as_bool(
            LaunchConfiguration('enable_marker_layer').perform(context),
            'enable_marker_layer',
        ),
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
            'enable_straight_line_planner',
            default_value='false',
            description=(
                'Load LabStraightLine and select the bundled straight-line BT. '
                'The official GridBased planner is used when false.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_observe_spin',
            default_value='false',
            description=(
                'Load ObserveSpin and select the bundled recovery BT. '
                'Official Nav2 recovery behaviors are used when false.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_marker_layer',
            default_value='false',
            description=(
                'Insert and enable the custom MarkerLayer in the global costmap.'
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
