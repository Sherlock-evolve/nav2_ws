import os
import unittest
from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'


class MissionConfigTest(unittest.TestCase):
    def test_all_mission_files_have_required_shape(self):
        mission_files = sorted(CONFIG_DIR.glob('*.yaml'))
        self.assertGreater(len(mission_files), 0)

        for mission_file in mission_files:
            with self.subTest(mission_file=os.path.basename(mission_file)):
                with open(mission_file, 'r', encoding='utf-8') as stream:
                    mission = yaml.safe_load(stream)

                self.assertIsInstance(mission, dict)
                self.assertEqual(mission.get('frame_id'), 'map')
                self.assertGreater(float(mission.get('default_timeout_sec', 0.0)), 0.0)
                self.assertGreaterEqual(int(mission.get('retry_count', 0)), 0)
                self.assert_pose_shape(mission.get('initial_pose'))

                goals = mission.get('goals')
                self.assertIsInstance(goals, list)
                self.assertGreater(len(goals), 0)
                for goal in goals:
                    self.assertIsInstance(goal.get('name'), str)
                    self.assert_pose_shape(goal)

    def test_simple_room_baseline_goal_order(self):
        with open(CONFIG_DIR / 'simple_room_mission.yaml', 'r', encoding='utf-8') as stream:
            mission = yaml.safe_load(stream)

        self.assertEqual(
            [goal['name'] for goal in mission['goals']],
            ['east_side', 'north_west', 'home'],
        )

    def assert_pose_shape(self, pose):
        self.assertIsInstance(pose, dict)
        for key in ('x', 'y', 'yaw'):
            self.assertIn(key, pose)
            self.assertIsInstance(float(pose[key]), float)


if __name__ == '__main__':
    unittest.main()
