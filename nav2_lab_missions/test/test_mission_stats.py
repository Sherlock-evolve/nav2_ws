import csv
import os
import tempfile
import unittest

from nav2_lab_missions.mission_stats import (
    check_baseline,
    compare_summaries,
    expand_inputs,
    format_comparison,
    format_run_details,
    format_summary,
    summarize_file,
    summarize_files,
)


def _write_csv(path, rows):
    with open(path, 'w', encoding='utf-8', newline='') as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                'goal_name',
                'attempt',
                'status',
                'duration_sec',
                'recovery_count',
                'message',
            ],
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
                    'recovery_count': '2',
                    'message': 'goal timed out',
                },
                {
                    'goal_name': 'east_side',
                    'attempt': '2',
                    'status': 'succeeded',
                    'duration_sec': '8.500',
                    'recovery_count': '1',
                    'message': '',
                },
                {
                    'goal_name': 'home',
                    'attempt': '1',
                    'status': 'succeeded',
                    'duration_sec': '6.000',
                    'recovery_count': '0',
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
        self.assertEqual(summary['total_recoveries'], 3)
        self.assertEqual(summary['final_recoveries'], {
            'east_side': 1,
            'home': 0,
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
        self.assertIn('Total recoveries: 0', text)
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
                    self._row('home', 'succeeded', '80.0', recovery_count='2'),
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
                'max_avg_final_recoveries': {
                    'home': 1.0,
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
            any(
                'home: avg_final_duration' in failure
                for failure in result['failures']
            )
        )
        self.assertTrue(
            any(
                'home: avg_final_recoveries' in failure
                for failure in result['failures']
            )
        )

    def test_format_comparison_reports_run_and_goal_deltas(self):
        """Comparison output includes aggregate and per-goal deltas."""
        with tempfile.TemporaryDirectory() as tmpdir:
            default_path = os.path.join(tmpdir, 'default_mission.csv')
            custom_path = os.path.join(tmpdir, 'custom_mission.csv')
            _write_csv(default_path, [
                self._row('east_side', 'succeeded', '10.0'),
                self._row('home', 'succeeded', '30.0'),
            ])
            _write_csv(custom_path, [
                self._row('east_side', 'succeeded', '12.0', recovery_count='2'),
                self._row('home', 'succeeded', '29.0', recovery_count='1'),
            ])

            comparison = compare_summaries(
                summarize_files([default_path]),
                summarize_files([custom_path]),
            )
            text = format_comparison(
                comparison,
                'default_bt',
                'goal_patience_bt',
            )

        self.assertIn('Comparison: default_bt -> goal_patience_bt', text)
        self.assertIn(
            'Successful runs: 1/1 (100.0%) -> 1/1 (100.0%)',
            text,
        )
        self.assertIn('Avg total duration: 40.000s -> 41.000s', text)
        self.assertIn('Avg recoveries/run: 0.000 -> 3.000', text)
        self.assertIn('east_side: 10.000s -> 12.000s', text)
        self.assertIn('home: 30.000s -> 29.000s', text)
        self.assertIn('Goal avg final recovery deltas:', text)
        self.assertIn('east_side: 0.000 -> 2.000', text)

    def test_format_run_details_reports_attempts_and_recoveries(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'failed_mission.csv')
            _write_csv(path, [
                self._row(
                    'blocked_east_approach',
                    'timeout',
                    '120.0',
                    attempt='1',
                    recovery_count='12',
                ),
                self._row(
                    'blocked_east_approach',
                    'timeout',
                    '120.0',
                    attempt='2',
                    recovery_count='11',
                ),
                self._row(
                    'home',
                    'succeeded',
                    '25.0',
                    recovery_count='1',
                ),
            ])

            text = format_run_details(summarize_files([path]))

        self.assertIn('failed_mission.csv: failed, attempts=3', text)
        self.assertIn('recoveries=24', text)
        self.assertIn(
            'blocked_east_approach: final=timeout, '
            'final_duration=120.000s, final_recoveries=11',
            text,
        )
        self.assertIn('a1:timeout 120.000s rec=12', text)
        self.assertIn('a2:timeout 120.000s rec=11', text)

    def _row(self, goal_name, status, duration, attempt='1', recovery_count='0'):
        return {
            'goal_name': goal_name,
            'attempt': attempt,
            'status': status,
            'duration_sec': duration,
            'recovery_count': recovery_count,
            'message': '',
        }


if __name__ == '__main__':
    unittest.main()
