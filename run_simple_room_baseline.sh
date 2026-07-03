#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./run_simple_room_baseline.sh [RUN_COUNT]

Run the simple_room mission repeatedly, then check the built-in baseline.

Environment:
  NAV2_LAB_SKIP_BUILD=1       Skip colcon build before running.
  NAV2_LAB_RESULTS_DIR=PATH   Override result directory. Defaults to /tmp/nav2_lab_results.
  ROS_LOG_DIR=PATH            Override ROS log directory. Defaults to /tmp/ros_logs.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

run_count="${1:-5}"
if ! [[ "${run_count}" =~ ^[0-9]+$ ]] || (( run_count < 5 )); then
  echo "RUN_COUNT must be an integer >= 5 for the simple_room baseline." >&2
  exit 2
fi

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
result_dir="${NAV2_LAB_RESULTS_DIR:-/tmp/nav2_lab_results}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"

cd "${workspace_dir}"
mkdir -p "${result_dir}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "${NAV2_LAB_SKIP_BUILD:-0}" != "1" ]]; then
  echo "[baseline] Building workspace"
  colcon build --symlink-install
fi

set +u
source "${workspace_dir}/install/setup.bash"
set -u

shopt -s nullglob
old_results=("${result_dir}"/*_mission.csv "${result_dir}"/*_telemetry.csv)
if (( ${#old_results[@]} > 0 )); then
  archive_dir="${result_dir}/archive/$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${archive_dir}"
  mv "${old_results[@]}" "${archive_dir}/"
  echo "[baseline] Archived ${#old_results[@]} old result file(s) to ${archive_dir}"
fi
shopt -u nullglob

for ((run_index = 1; run_index <= run_count; run_index++)); do
  echo "[baseline] Starting simple_room run ${run_index}/${run_count}"
  ros2 launch nav2_lab_bringup lab.launch.py \
    world:=simple_room \
    map:=simple_room \
    mission:=simple_room_mission \
    run_mission:=true \
    shutdown_on_mission_complete:=true \
    use_rviz:=false \
    use_gzclient:=false
  echo "[baseline] Finished simple_room run ${run_index}/${run_count}"
  sleep 2
done

echo "[baseline] Checking simple_room baseline"
ros2 run nav2_lab_missions mission_stats "${result_dir}" --baseline simple_room
