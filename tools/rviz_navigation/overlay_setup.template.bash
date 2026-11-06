#!/usr/bin/env bash
# Source this from a reviewed local stub; contains no machine-specific defaults.
cleannav_load_reviewed_rviz_overlay() {
  local cfg=${CLEANNAV_RVIZ_MACHINE_CONFIG:-}
  local helper_dir
  helper_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
  [[ -n "$cfg" && -f "$cfg" ]] || { printf 'CLEANNAV_RVIZ_MACHINE_CONFIG must identify a reviewed local JSON file\n' >&2; return 2; }
  python3 "$helper_dir/overlay_identity.py" check-inputs --config "$cfg" || return
  local -a values=()
  mapfile -t values < <(python3 "$helper_dir/overlay_identity.py" values --config "$cfg")
  [[ ${#values[@]} == 8 ]] || { printf 'invalid reviewed overlay values\n' >&2; return 2; }
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  # Exactly the audited success source chain: base setup, then its existing
  # environment entry. Prefixes are bindings, never a list of sources to run.
  source "${values[0]}" || return
  source "${values[1]}" || return
  export CLEANNAV_EXPECTED_INTERFACES_PREFIX="${values[2]}"
  export CLEANNAV_EXPECTED_MISSION_MANAGER_PREFIX="${values[3]}"
  export CLEANNAV_EXPECTED_NAVIGATION_PREFIX="${values[4]}"
  export CLEANNAV_EXPECTED_RTABMAP_PREFIX="${values[5]}"
  export CLEANNAV_EXPECTED_SAFETY_SUPERVISOR_PREFIX="${values[6]}"
  export CLEANNAV_EXPECTED_SIMULATION_PREFIX="${values[7]}"
  python3 "$helper_dir/overlay_identity.py" check-environment --config "$cfg"
}
if cleannav_load_reviewed_rviz_overlay; then
  unset -f cleannav_load_reviewed_rviz_overlay
else
  cleannav_overlay_status=$?
  unset -f cleannav_load_reviewed_rviz_overlay
  return "$cleannav_overlay_status" 2>/dev/null || exit "$cleannav_overlay_status"
fi
