import importlib.util
import os
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PARAMS_FILE = PACKAGE_DIR / 'params' / 'nav2_params.yaml'
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'navigation.launch.py'


def _load_navigation_launch():
    spec = importlib.util.spec_from_file_location('nav2_lab_navigation_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_params(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_default_params_load_only_official_plugins():
    params = _load_params(PARAMS_FILE)

    planner_plugins = params['planner_server']['ros__parameters']['planner_plugins']
    behavior_plugins = params['behavior_server']['ros__parameters']['behavior_plugins']
    costmap_params = params['global_costmap']['global_costmap']['ros__parameters']

    assert planner_plugins == ['GridBased']
    assert 'LabStraightLine' not in planner_plugins
    assert 'observe_spin' not in behavior_plugins
    assert 'lab_marker_layer' not in costmap_params['plugins']
    assert costmap_params['lab_marker_layer']['enabled'] is False


def test_all_switches_load_all_custom_plugins_and_combined_bt():
    launch_module = _load_navigation_launch()
    generated = launch_module._params_file_with_plugin_selection(
        str(PARAMS_FILE),
        '',
        str(PACKAGE_DIR),
        enable_straight_line_planner=True,
        enable_observe_spin=True,
        enable_marker_layer=True,
    )

    try:
        params = _load_params(generated)
        planner_plugins = params['planner_server']['ros__parameters']['planner_plugins']
        behavior_plugins = params['behavior_server']['ros__parameters']['behavior_plugins']
        costmap_params = params['global_costmap']['global_costmap']['ros__parameters']
        bt_xml = params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']

        assert planner_plugins == ['GridBased', 'LabStraightLine']
        assert behavior_plugins[-1] == 'observe_spin'
        assert costmap_params['plugins'][-2:] == ['lab_marker_layer', 'inflation_layer']
        assert costmap_params['lab_marker_layer']['enabled'] is True
        assert bt_xml.endswith('nav2_lab_all_plugins.xml')
        assert os.path.exists(bt_xml)
    finally:
        os.unlink(generated)


def test_bundled_bt_shortcut_still_loads_its_plugin():
    launch_module = _load_navigation_launch()
    generated = launch_module._params_file_with_plugin_selection(
        str(PARAMS_FILE),
        'nav2_lab_recovery',
        str(PACKAGE_DIR),
    )

    try:
        params = _load_params(generated)
        behavior_plugins = params['behavior_server']['ros__parameters']['behavior_plugins']
        assert behavior_plugins[-1] == 'observe_spin'
    finally:
        os.unlink(generated)
