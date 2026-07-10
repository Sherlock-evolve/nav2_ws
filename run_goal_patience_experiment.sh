#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: ./run_goal_patience_experiment.sh [RUN_COUNT]

Run the goal_patience mission repeatedly for BT behavior-tree comparison.

Environment:
  NAV2_LAB_SKIP_BUILD=1       Skip colcon build before running.
  NAV2_LAB_RESULTS_DIR=PATH   Override result directory. Defaults to /tmp/nav2_lab_results.
  NAV2_LAB_EXPERIMENT=NAME    Label this result batch.
  NAV2_LAB_BT_XML=NAME_OR_PATH Override NavigateToPose BT XML for all runs.
  ROS_LOG_DIR=PATH            Override ROS log directory. Defaults to /tmp/ros_logs.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

run_count="${1:-5}"
if ! [[ "${run_count}" =~ ^[0-9]+$ ]] || (( run_count < 1 )); then
  echo "RUN_COUNT must be an integer >= 1." >&2
  exit 2
fi

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
result_dir="${NAV2_LAB_RESULTS_DIR:-/tmp/nav2_lab_results}"
bt_xml="${NAV2_LAB_BT_XML:-}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"

sanitize_label() {
  local value="${1:-experiment}"
  value="${value##*/}"
  value="${value%.xml}"
  value="${value//[^[:alnum:]_.-]/_}"
  printf '%s' "${value:-experiment}"
}

raw_experiment="${NAV2_LAB_EXPERIMENT:-}"
if [[ -z "${raw_experiment}" ]]; then
  if [[ -n "${bt_xml}" ]]; then
    raw_experiment="goal_patience_bt_$(sanitize_label "${bt_xml}")"
  else
    raw_experiment="goal_patience_default_bt"
  fi
fi
experiment="$(sanitize_label "${raw_experiment}")"
metadata_file="${result_dir}/metadata.env"

cd "${workspace_dir}"
mkdir -p "${result_dir}" "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ "${NAV2_LAB_SKIP_BUILD:-0}" != "1" ]]; then
  echo "[goal_patience] Building workspace"
  colcon build --symlink-install
fi

set +u
source "${workspace_dir}/install/setup.bash"
set -u

shopt -s nullglob
old_results=("${result_dir}"/*_mission.csv "${result_dir}"/*_telemetry.csv)
if [[ -f "${metadata_file}" ]]; then
  old_results+=("${metadata_file}")
fi
if (( ${#old_results[@]} > 0 )); then
  old_experiment=""
  if [[ -f "${metadata_file}" ]]; then
    while IFS='=' read -r key value; do
      if [[ "${key}" == "experiment" ]]; then
        old_experiment="${value}"
        break
      fi
    done < "${metadata_file}"
  fi
  archive_label="$(sanitize_label "${old_experiment:-previous}")"
  archive_dir="${result_dir}/archive/$(date +%Y%m%d_%H%M%S)_${archive_label}"
  mkdir -p "${archive_dir}"
  mv "${old_results[@]}" "${archive_dir}/"
  echo "[goal_patience] Archived ${#old_results[@]} old result file(s) to ${archive_dir}"
fi
shopt -u nullglob

{
  printf 'experiment=%s\n' "${experiment}"
  printf 'world=goal_patience\n'
  printf 'map=simple_room\n'
  printf 'mission=goal_patience_mission\n'
  printf 'bt_xml=%s\n' "${bt_xml}"
  printf 'run_count=%s\n' "${run_count}"
  printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
} > "${metadata_file}"

echo "[goal_patience] Experiment: ${experiment}"

bt_launch_args=()
if [[ -n "${bt_xml}" ]]; then
  bt_launch_args=("bt_xml:=${bt_xml}")
  echo "[goal_patience] Using custom BT XML: ${bt_xml}"
fi

for ((run_index = 1; run_index <= run_count; run_index++)); do
  echo "[goal_patience] Starting run ${run_index}/${run_count}"
  ros2 launch nav2_lab_bringup lab.launch.py \
    world:=goal_patience \
    map:=simple_room \
    mission:=goal_patience_mission \
    run_mission:=true \
    shutdown_on_mission_complete:=true \
    use_rviz:=false \
    use_gzclient:=false \
    "${bt_launch_args[@]}"
  echo "[goal_patience] Finished run ${run_index}/${run_count}"
  sleep 2
done

printf 'completed_at=%s\n' "$(date --iso-8601=seconds)" >> "${metadata_file}"

echo "[goal_patience] Summarizing mission results"
ros2 run nav2_lab_missions mission_stats "${result_dir}" --require-success
