import csv
import os
import tempfile
import unittest

from nav2_lab_missions.mission_stats import (
    check_baseline,
    expand_inputs,
    format_summary,
    summarize_file,
    summarize_files,
)


def _write_csv(path, rows):
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=['goal_name', 'attempt', 'status', 'duration_sec', 'message'],
        )
        writer.writeheader()
        writer.writerows(rows)


class MissionStatsTest(unittest.TestCase):
    def test_summarize_file_uses_last_goal_attempt_as_final_status(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'run_mission.csv')
            _write_csv(path, [
                {
                    'goal_name': 'east_side',
                    'attempt': '1',
                    'status': 'timeout',
                    'duration_sec': '90.000',
                    'message': 'goal timed out',
                },
                {
                    'goal_name': 'east_side',
                    'attempt': '2',
                    'status': 'succeeded',
                    'duration_sec': '8.500',
                    'message': '',
                },
                {
                    'goal_name': 'home',
                    'attempt': '1',
                    'status': 'succeeded',
                    'duration_sec': '6.000',
                    'message': '',
                },
            ])

            summary = summarize_file(path)

        self.assertTrue(summary['success'])
        self.assertEqual(summary['attempt_count'], 3)
        self.assertEqual(summary['goal_count'], 2)
        self.assertEqual(summary['status_counts']['timeout'], 1)
        self.assertEqual(summary['final_statuses'], {
            'east_side': 'succeeded',
            'home': 'succeeded',
        })

    def test_summarize_files_reports_failed_runs(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            success_path = os.path.join(tmpdir, 'success_mission.csv')
            failed_path = os.path.join(tmpdir, 'failed_mission.csv')
            _write_csv(success_path, [
                {
                    'goal_name': 'east_side',
                    'attempt': '1',
                    'status': 'succeeded',
                    'duration_sec': '5.000',
                    'message': '',
                },
            ])
            _write_csv(failed_path, [
                {
                    'goal_name': 'east_side',
                    'attempt': '1',
                    'status': 'aborted',
                    'duration_sec': '3.000',
                    'message': 'planner failed',
                },
            ])

            summary = summarize_files([failed_path, success_path])
            text = format_summary(summary)

        self.assertEqual(summary['run_count'], 2)
        self.assertEqual(summary['successful_runs'], 1)
        self.assertIn('Successful runs: 1/2 (50.0%)', text)
        self.assertIn('Failed runs:', text)

    def test_expand_inputs_accepts_directories_and_globs(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            wanted = os.path.join(tmpdir, 'one_mission.csv')
            ignored = os.path.join(tmpdir, 'telemetry.csv')
            open(wanted, 'w', encoding='utf-8').close()
            open(ignored, 'w', encoding='utf-8').close()

            self.assertEqual(expand_inputs([tmpdir]), [wanted])
            self.assertEqual(expand_inputs([os.path.join(tmpdir, '*_mission.csv')]), [wanted])

    def test_check_baseline_passes_clean_simple_room_runs(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = []
            for index in range(5):
                path = os.path.join(tmpdir, f'run_{index}_mission.csv')
                _write_csv(path, [
                    self._row('east_side', 'succeeded', '10.0'),
                    self._row('north_west', 'succeeded', '20.0'),
                    self._row('home', 'succeeded', '30.0'),
                ])
                paths.append(path)

            summary = summarize_files(paths)
            result = check_baseline(summary, {
                'min_run_count': 5,
                'required_success_rate': 1.0,
                'allowed_statuses': ['succeeded'],
                'expected_goals': ['east_side', 'north_west', 'home'],
                'max_attempts_per_run': 3,
                'max_avg_total_duration_sec': 90.0,
                'max_avg_final_duration_sec': {
                    'east_side': 25.0,
                    'north_west': 45.0,
                    'home': 70.0,
                },
            })

        self.assertTrue(result['passed'], result['failures'])

    def test_check_baseline_fails_retries_and_slow_goals(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = []
            for index in range(5):
                path = os.path.join(tmpdir, f'run_{index}_mission.csv')
                _write_csv(path, [
                    self._row('east_side', 'timeout', '90.0', attempt='1'),
                    self._row('east_side', 'succeeded', '30.0', attempt='2'),
                    self._row('north_west', 'succeeded', '20.0'),
                    self._row('home', 'succeeded', '80.0'),
                ])
                paths.append(path)

            summary = summarize_files(paths)
            result = check_baseline(summary, {
                'min_run_count': 5,
                'required_success_rate': 1.0,
                'allowed_statuses': ['succeeded'],
                'expected_goals': ['east_side', 'north_west', 'home'],
                'max_attempts_per_run': 3,
                'max_avg_final_duration_sec': {
                    'home': 70.0,
                },
            })

        self.assertFalse(result['passed'])
        self.assertTrue(
            any('unexpected statuses' in failure for failure in result['failures'])
        )
        self.assertTrue(
            any('total_attempts' in failure for failure in result['failures'])
        )
        self.assertTrue(
            any('home: avg_final_duration' in failure for failure in result['failures'])
        )

    def _row(self, goal_name, status, duration, attempt='1'):
        return {
            'goal_name': goal_name,
            'attempt': attempt,
            'status': status,
            'duration_sec': duration,
            'message': '',
        }


if __name__ == '__main__':
    unittest.main()
