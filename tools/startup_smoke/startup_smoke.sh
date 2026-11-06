#!/usr/bin/env bash
# Formal no-motion startup entry. Run as a child process; it never execs the caller shell.
set -eo pipefail
ENTRY_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
NAV_ROOT=$(cd -- "$ENTRY_DIR/../.." && pwd)
WORKSPACE_ROOT=$NAV_ROOT
while [[ ! -f "$WORKSPACE_ROOT/cleannav-build/locks/build.lock" || ! -f "$WORKSPACE_ROOT/cleannav-build/locks/runtime.lock" ]]; do
  [[ "$WORKSPACE_ROOT" != / ]] || { printf 'CleanNav workspace root not found from Navigation checkout\n' >&2; exit 2; }
  WORKSPACE_ROOT=$(dirname -- "$WORKSPACE_ROOT")
done
cd -- "$WORKSPACE_ROOT"
export CLEANNAV_WORKSPACE_ROOT="$WORKSPACE_ROOT"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash
command -v ros2
command -v xacro
: "${CLEANNAV_STARTUP_OVERLAY_SETUP:?set CLEANNAV_STARTUP_OVERLAY_SETUP to the approved local ROS overlay setup script}"
[[ -f "$CLEANNAV_STARTUP_OVERLAY_SETUP" ]] || { printf 'overlay setup file does not exist\n' >&2; exit 2; }
source "$CLEANNAV_STARTUP_OVERLAY_SETUP"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11704 GAZEBO_IP=127.0.0.1 GAZEBO_MODEL_DATABASE_URI=''
set +e
python3 "$ENTRY_DIR/startup_supervisor.py" "$@"
rc=$?
printf 'startup_smoke exit_code=%s\n' "$rc"
exit "$rc"
