import argparse
import csv
import glob
import os
from collections import Counter, OrderedDict, defaultdict

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


SUCCESS_STATUS = 'succeeded'
DEFAULT_PATTERN = '/tmp/nav2_lab_results/*_mission.csv'


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def expand_inputs(inputs):
    """Resolve files, directories, and glob patterns to sorted CSV paths."""
    patterns = inputs or [DEFAULT_PATTERN]
    paths = []
    for pattern in patterns:
        expanded = os.path.expanduser(pattern)
        if os.path.isdir(expanded):
            paths.extend(glob.glob(os.path.join(expanded, '*_mission.csv')))
        elif glob.has_magic(expanded):
            paths.extend(glob.glob(expanded))
        else:
            paths.append(expanded)
    return sorted(dict.fromkeys(paths))


def summarize_file(path):
    """Summarize one mission_runner CSV.

    The final status of a goal is the last row written for that goal, which
    accounts for retries.
    """
    rows = []
    with open(path, 'r', encoding='utf-8', newline='') as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if row.get('goal_name'):
                rows.append(row)

    final_rows = OrderedDict()
    for row in rows:
        final_rows[row['goal_name']] = row

    final_statuses = {
        goal_name: row.get('status', '')
        for goal_name, row in final_rows.items()
    }
    final_durations = {
        goal_name: _as_float(row.get('duration_sec'))
        for goal_name, row in final_rows.items()
    }
    total_duration = sum(_as_float(row.get('duration_sec')) for row in rows)
    success = bool(final_statuses) and all(
        status == SUCCESS_STATUS for status in final_statuses.values()
    )

    return {
        'path': path,
        'rows': rows,
        'attempt_count': len(rows),
        'goal_count': len(final_rows),
        'success': success,
        'total_duration_sec': total_duration,
        'status_counts': Counter(row.get('status', '') for row in rows),
        'final_statuses': final_statuses,
        'final_durations_sec': final_durations,
    }


def summarize_files(paths):
    file_summaries = [summarize_file(path) for path in paths]
    status_counts = Counter()
    final_by_goal = defaultdict(Counter)
    duration_by_goal = defaultdict(list)

    for summary in file_summaries:
        status_counts.update(summary['status_counts'])
        for goal_name, status in summary['final_statuses'].items():
            final_by_goal[goal_name][status] += 1
            duration_by_goal[goal_name].append(
                summary['final_durations_sec'][goal_name]
            )

    run_count = len(file_summaries)
    successful_runs = sum(1 for summary in file_summaries if summary['success'])
    total_attempts = sum(summary['attempt_count'] for summary in file_summaries)
    total_duration = sum(summary['total_duration_sec'] for summary in file_summaries)

    return {
        'files': file_summaries,
        'run_count': run_count,
        'successful_runs': successful_runs,
        'success_rate': 0.0 if run_count == 0 else successful_runs / run_count,
        'total_attempts': total_attempts,
        'total_duration_sec': total_duration,
        'status_counts': status_counts,
        'final_by_goal': final_by_goal,
        'duration_by_goal': duration_by_goal,
    }


def resolve_baseline(value):
    """Resolve a baseline name or YAML path."""
    expanded = os.path.expanduser(value)
    if os.path.isabs(expanded) or os.path.exists(expanded):
        return expanded

    candidate = expanded
    if not candidate.endswith('.yaml'):
        candidate = f'{candidate}.yaml'

    try:
        package_share = get_package_share_directory('nav2_lab_missions')
        installed = os.path.join(package_share, 'baselines', candidate)
        if os.path.exists(installed):
            return installed
    except PackageNotFoundError:
        pass

    source_package = os.path.dirname(os.path.dirname(__file__))
    return os.path.join(source_package, 'baselines', candidate)


def load_baseline(path):
    with open(path, 'r', encoding='utf-8') as stream:
        baseline = yaml.safe_load(stream) or {}
    if not isinstance(baseline, dict):
        raise ValueError(f'Baseline must be a mapping: {path}')
    return baseline


def check_baseline(summary, baseline):
    failures = []
    details = []
    run_count = summary['run_count']

    min_run_count = int(baseline.get('min_run_count', 1))
    if run_count < min_run_count:
        failures.append(f'run_count {run_count} < required {min_run_count}')

    required_success_rate = float(baseline.get('required_success_rate', 1.0))
    if summary['success_rate'] < required_success_rate:
        failures.append(
            f'success_rate {summary["success_rate"]:.3f} '
            f'< required {required_success_rate:.3f}'
        )

    allowed_statuses = set(baseline.get('allowed_statuses', [SUCCESS_STATUS]))
    observed_statuses = set(summary['status_counts'])
    unexpected_statuses = sorted(observed_statuses - allowed_statuses)
    if unexpected_statuses:
        failures.append(f'unexpected statuses: {", ".join(unexpected_statuses)}')

    expected_goals = baseline.get('expected_goals', [])
    if expected_goals:
        expected_goal_set = set(expected_goals)
        observed_goal_set = set(summary['final_by_goal'])
        missing = sorted(expected_goal_set - observed_goal_set)
        extra = sorted(observed_goal_set - expected_goal_set)
        if missing:
            failures.append(f'missing goals: {", ".join(missing)}')
        if extra:
            failures.append(f'unexpected goals: {", ".join(extra)}')

        for file_summary in summary['files']:
            file_goals = set(file_summary['final_statuses'])
            if file_goals != expected_goal_set:
                failures.append(
                    f'{file_summary["path"]}: goal set '
                    f'{sorted(file_goals)} != {sorted(expected_goal_set)}'
                )

    max_attempts_per_run = baseline.get('max_attempts_per_run')
    if max_attempts_per_run is not None:
        max_total_attempts = int(max_attempts_per_run) * run_count
        if summary['total_attempts'] > max_total_attempts:
            failures.append(
                f'total_attempts {summary["total_attempts"]} '
                f'> allowed {max_total_attempts}'
            )

    max_avg_total_duration = baseline.get('max_avg_total_duration_sec')
    if max_avg_total_duration is not None and run_count > 0:
        avg_total_duration = summary['total_duration_sec'] / run_count
        limit = float(max_avg_total_duration)
        if avg_total_duration > limit:
            failures.append(
                f'avg_total_duration {avg_total_duration:.3f}s > {limit:.3f}s'
            )
        else:
            details.append(
                f'avg_total_duration {avg_total_duration:.3f}s <= {limit:.3f}s'
            )

    max_goal_durations = baseline.get('max_avg_final_duration_sec', {})
    for goal_name, limit_value in max_goal_durations.items():
        durations = summary['duration_by_goal'].get(goal_name, [])
        if not durations:
            failures.append(f'{goal_name}: no duration samples')
            continue
        avg_duration = sum(durations) / len(durations)
        limit = float(limit_value)
        if avg_duration > limit:
            failures.append(
                f'{goal_name}: avg_final_duration '
                f'{avg_duration:.3f}s > {limit:.3f}s'
            )
        else:
            details.append(
                f'{goal_name}: avg_final_duration '
                f'{avg_duration:.3f}s <= {limit:.3f}s'
            )

    return {
        'passed': not failures,
        'failures': failures,
        'details': details,
    }


def format_baseline_result(result, baseline_name):
    status = 'PASS' if result['passed'] else 'FAIL'
    lines = [f'Baseline check ({baseline_name}): {status}']
    for detail in result['details']:
        lines.append(f'  OK: {detail}')
    for failure in result['failures']:
        lines.append(f'  FAIL: {failure}')
    return '\n'.join(lines)


def format_summary(summary):
    lines = []
    run_count = summary['run_count']
    successful_runs = summary['successful_runs']
    success_rate = summary['success_rate'] * 100.0

    lines.append(f'Mission files: {run_count}')
    lines.append(f'Successful runs: {successful_runs}/{run_count} ({success_rate:.1f}%)')
    lines.append(f'Total attempts: {summary["total_attempts"]}')
    lines.append(f'Total recorded duration: {summary["total_duration_sec"]:.3f}s')

    if summary['status_counts']:
        lines.append('Status counts:')
        for status, count in sorted(summary['status_counts'].items()):
            lines.append(f'  {status}: {count}')

    if summary['final_by_goal']:
        lines.append('Final goal results:')
        for goal_name in sorted(summary['final_by_goal']):
            counts = summary['final_by_goal'][goal_name]
            durations = summary['duration_by_goal'][goal_name]
            succeeded = counts.get(SUCCESS_STATUS, 0)
            total = sum(counts.values())
            avg_duration = sum(durations) / len(durations)
            status_text = ', '.join(
                f'{status}={count}' for status, count in sorted(counts.items())
            )
            lines.append(
                f'  {goal_name}: {succeeded}/{total} succeeded, '
                f'avg_final_duration={avg_duration:.3f}s ({status_text})'
            )

    failed_files = [
        file_summary for file_summary in summary['files']
        if not file_summary['success']
    ]
    if failed_files:
        lines.append('Failed runs:')
        for file_summary in failed_files:
            statuses = ', '.join(
                f'{goal}={status}'
                for goal, status in file_summary['final_statuses'].items()
            )
            lines.append(f'  {file_summary["path"]}: {statuses}')

    return '\n'.join(lines)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Summarize nav2_lab mission_runner CSV results.'
    )
    parser.add_argument(
        'paths',
        nargs='*',
        help=(
            'CSV files, directories, or glob patterns. Defaults to '
            f'{DEFAULT_PATTERN}.'
        ),
    )
    parser.add_argument(
        '--require-success',
        action='store_true',
        help='Exit with status 1 if any mission file did not finish successfully.',
    )
    parser.add_argument(
        '--baseline',
        help=(
            'Baseline YAML path or name under nav2_lab_missions/baselines. '
            'Example: --baseline simple_room'
        ),
    )
    parsed = parser.parse_args(args=args)

    paths = expand_inputs(parsed.paths)
    if not paths:
        print(f'No mission CSV files found for: {", ".join(parsed.paths or [DEFAULT_PATTERN])}')
        return 2

    missing = [path for path in paths if not os.path.exists(path)]
    if missing:
        print('Missing mission CSV files:')
        for path in missing:
            print(f'  {path}')
        return 2

    summary = summarize_files(paths)
    print(format_summary(summary))

    baseline_result = None
    if parsed.baseline:
        baseline_path = resolve_baseline(parsed.baseline)
        if not os.path.exists(baseline_path):
            print(f'Baseline file not found: {baseline_path}')
            return 2
        baseline = load_baseline(baseline_path)
        baseline_result = check_baseline(summary, baseline)
        print(format_baseline_result(baseline_result, parsed.baseline))

    if parsed.require_success and summary['successful_runs'] != summary['run_count']:
        return 1
    if baseline_result is not None and not baseline_result['passed']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
